//! Layer-level geometry checks over already-flattened regiones.
//!
//! These checks are independent of KiCad concepts such as nets and drills, so
//! Gerber-derived layers and KiCad-derived layers share the same behavior.
//!
//! Reliability note: offset/opening operations, local-density windows, and
//! boundary-distance fallbacks approximate manufacturing concerns over polygon
//! data. Double-check suspect results against CAM tooling when geometry is
//! self-touching, highly fragmented, or near rule thresholds.

use std::collections::BTreeMap;

use hypercurve::{CurvePolicy, CurveRegion2, LineLineIntersection, LineSeg2};
use hyperlimit::{Point2, PredicatePolicy, SegmentIntersection, Sign, compare_reals_with_policy};

use crate::checks::distance::polygon_boundary_distance_scalar_with_grid;
use crate::checks::spatial::LayerPolygonSpatialIndex;
use crate::checks::{
    difference_for_check, intersection_for_check, offset_for_check, union_for_check,
};
use crate::geometry::{
    Coord, LineString, MultiPolygon, Polygon, RuleGeometryProvenance, SourceGridFacts,
    balanced_scalar_sum, multipolygon_area_scalar, multipolygon_to_shapes_scalar,
    polygon_area_scalar, polygon_bounds_scalar, polygon_to_profile, polygons_to_profile,
    rect_polygon,
};
use crate::ipc356::Ipc356Point;
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbRegion, PcbRegionExt, Scalar};

const DUPLICATE_LAYER_OVERLAP_RATIO: &str = "0.999999";
const DUPLICATE_LAYER_SIGNATURE_SCALE: f64 = 1_000_000.0;

/// Run the `mask_island_keepout` design-readiness check or report helper.
///
/// Mask-island neighbors use `LayerPolygonSpatialIndex` before exact expanded
/// island intersection. The grid is only a broad phase: bbox-center candidates are never reported
/// until the offset CSG predicate produces non-trivial shapes.
pub fn mask_island_keepout(
    layer_name: &str,
    region: &PcbRegion,
    keepout: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let polygons = region.to_multipolygon().0;
    let doubled_keepout = keepout * crate::scalar::scalar("2");
    let broad_phase_keepout = scalar_broad_phase_radius(&doubled_keepout);
    let index = LayerPolygonSpatialIndex::new(&polygons, broad_phase_keepout);
    log::trace!(
        "mask-island keepout: layer={layer_name} islands={} buckets={} keepout={keepout:.6}",
        polygons.len(),
        index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_pairs = 0usize;

    for island_index in 0..polygons.len() {
        let candidate_indexes = index.later_candidates_near(island_index, broad_phase_keepout);
        candidate_pairs += candidate_indexes.len();
        for neighbor_index in candidate_indexes {
            if !polygons_within_clearance(
                &polygons[island_index],
                &polygons[neighbor_index],
                broad_phase_keepout,
            ) {
                continue;
            }

            // Offset both candidate islands and intersect only after a bounding
            // box broad phase. This avoids rebuilding and offsetting the rest
            // of a dense soldermask layer for every island while preserving the
            // same exact polygon predicate for nearby island pairs.
            let island =
                polygon_to_profile(polygons[island_index].clone(), Some(metadata(layer_name)));
            let neighbor =
                polygon_to_profile(polygons[neighbor_index].clone(), Some(metadata(layer_name)));
            let layers = vec![layer_name.to_string()];
            let expanded_island = match offset_for_check(
                &island,
                keepout.clone(),
                "mask-island-keepout",
                layers.clone(),
            ) {
                Ok(expanded) => expanded,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let expanded_neighbor =
                match offset_for_check(&neighbor, keepout.clone(), "mask-island-keepout", layers) {
                    Ok(expanded) => expanded,
                    Err(uncertainty) => return vec![*uncertainty],
                };
            let overlap = match intersection_for_check(
                &expanded_island,
                &expanded_neighbor,
                "mask-island-keepout",
                vec![layer_name.to_string()],
            ) {
                Ok(overlap) => overlap,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);

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

    log::trace!(
        "mask-island keepout finished: layer={layer_name} candidate_pairs={} violations={}",
        candidate_pairs,
        violations.len()
    );

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
    left: &PcbRegion,
    right_name: &str,
    right: &PcbRegion,
    min_area: &Scalar,
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

/// Run a copper-overlap check that uses IPC-D-356 net evidence when available.
pub fn copper_overlap_with_ipc356(
    left_name: &str,
    left: &PcbRegion,
    right_name: &str,
    right: &PcbRegion,
    ipc356_points: &[Ipc356Point],
    min_area: &Scalar,
) -> Vec<Violation> {
    let overlap = match intersection_for_check(
        left,
        right,
        "copper-overlap",
        vec![left_name.to_string(), right_name.to_string()],
    ) {
        Ok(overlap) => overlap,
        Err(uncertainty) => return vec![*uncertainty],
    };
    let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
    if shapes.is_empty() {
        return Vec::new();
    }

    let nets = ipc356_nets_in_region(&overlap, ipc356_points);
    let (severity, message) = match nets.as_slice() {
        [net] => (
            Severity::Warning,
            format!(
                "copper regions overlap across layers with IPC-D-356 same-net evidence for {net}; review whether the overlap is intentional"
            ),
        ),
        [] => (
            Severity::Error,
            "copper regions overlap across layers without IPC-D-356 same-net evidence".to_string(),
        ),
        _ => (
            Severity::Error,
            format!(
                "copper regions overlap across layers with mixed IPC-D-356 net evidence: {}",
                nets.join(", ")
            ),
        ),
    };

    vec![Violation::new(
        "copper-overlap",
        severity,
        vec![left_name.to_string(), right_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some(message),
    )]
}

fn ipc356_nets_in_region(overlap: &PcbRegion, ipc356_points: &[Ipc356Point]) -> Vec<String> {
    let mut nets = std::collections::BTreeSet::new();
    for point in ipc356_points {
        let net = point.net.trim();
        if net.is_empty() {
            continue;
        }
        if overlap.contains_xy(point.location[0].clone(), point.location[1].clone()) == Some(true) {
            nets.insert(net.to_string());
        }
    }
    nets.into_iter().collect()
}

/// Run the `board_edge_clearance` design-readiness check or report helper.
pub fn board_edge_clearance(
    copper_name: &str,
    copper: &PcbRegion,
    board_name: &str,
    board: &PcbRegion,
    clearance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let allowed = match offset_for_check(
        board,
        -clearance.clone(),
        "copper-to-board-edge-clearance",
        vec![copper_name.to_string(), board_name.to_string()],
    ) {
        Ok(allowed) => allowed,
        Err(uncertainty) => return vec![*uncertainty],
    };
    let intrusion = match difference_for_check(
        copper,
        &allowed,
        "copper-to-board-edge-clearance",
        vec![copper_name.to_string(), board_name.to_string()],
    ) {
        Ok(intrusion) => intrusion,
        Err(uncertainty) => return vec![*uncertainty],
    };
    let shapes = multipolygon_to_shapes_scalar(&intrusion.to_multipolygon(), min_area);

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
    subject: &PcbRegion,
    outline_name: &str,
    outline: &PcbRegion,
    clearance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    board_outline_cutout_clearance_with_grid(
        subject_name,
        subject,
        outline_name,
        outline,
        clearance,
        min_area,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Run cutout clearance with retained source-grid facts for exact boundary predicates.
pub fn board_outline_cutout_clearance_with_grid(
    subject_name: &str,
    subject: &PcbRegion,
    outline_name: &str,
    outline: &PcbRegion,
    clearance: &Scalar,
    min_area: &Scalar,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let outline_polygons = outline.to_multipolygon();
    let cutouts = match board_outline_cutouts(
        &outline_polygons,
        "board-outline-cutout-clearance",
        vec![subject_name.to_string(), outline_name.to_string()],
    ) {
        Ok(cutouts) => cutouts,
        Err(uncertainty) => return vec![*uncertainty],
    };
    for cutout in cutouts {
        let cutout = polygon_to_profile(cutout, Some(metadata("board cutout")));
        let clearance_band = if crate::scalar::gt(clearance, &Scalar::zero()) {
            match offset_for_check(
                &cutout,
                clearance.clone(),
                "board-outline-cutout-clearance",
                vec![subject_name.to_string(), outline_name.to_string()],
            ) {
                Ok(clearance_band) => clearance_band,
                Err(uncertainty) => return vec![*uncertainty],
            }
        } else {
            cutout.clone()
        };

        let intrusion = match intersection_for_check(
            subject,
            &clearance_band,
            "board-outline-cutout-clearance",
            vec![subject_name.to_string(), outline_name.to_string()],
        ) {
            Ok(intrusion) => intrusion,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let shapes = multipolygon_to_shapes_scalar(&intrusion.to_multipolygon(), min_area);
        let touches_cutout = shapes.is_empty()
            && polygon_boundary_distance_scalar_with_grid(
                &subject.to_multipolygon(),
                &cutout.to_multipolygon(),
                grid,
            )
            .is_some_and(|distance| crate::scalar::le(&distance, clearance));
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

fn board_outline_cutouts(
    outline: &MultiPolygon<f64>,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<Vec<Polygon<f64>>, Box<Violation>> {
    let polygons = &outline.0;
    if polygons.len() < 2 {
        return Ok(Vec::new());
    }

    let mut cutouts = Vec::new();
    for inner_index in 0..polygons.len() {
        let inner = &polygons[inner_index];
        if polygon_area_scalar(inner).is_none_or(|area| crate::scalar::le(&area, &Scalar::zero())) {
            continue;
        }

        let mut is_nested = false;
        for (outer_index, outer) in polygons.iter().enumerate() {
            if outer_index == inner_index {
                continue;
            }
            if polygon_contains_other_outer(
                outer,
                inner,
                &crate::scalar::scalar(BOARD_OUTLINE_NESTED_OVERLAP_RATIO),
                &crate::scalar::scalar(BOARD_OUTLINE_GEOMETRY_TOLERANCE),
                requested_check,
                layers.clone(),
            )? {
                is_nested = true;
                break;
            }
        }
        if !is_nested {
            continue;
        }

        let Some(point) = representative_point(inner) else {
            continue;
        };
        if cutouts
            .iter()
            .filter_map(representative_point)
            .any(|candidate| locations_are_equal(&candidate, &point))
        {
            continue;
        }

        cutouts.push(inner.clone());
    }

    Ok(cutouts)
}

/// Run the `silkscreen_board_edge_clearance` design-readiness check or report helper.
pub fn silkscreen_board_edge_clearance(
    silk_name: &str,
    silk: &PcbRegion,
    board_name: &str,
    board: &PcbRegion,
    clearance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let allowed = match offset_for_check(
        board,
        -clearance.clone(),
        "silkscreen-to-board-edge-clearance",
        vec![silk_name.to_string(), board_name.to_string()],
    ) {
        Ok(allowed) => allowed,
        Err(uncertainty) => return vec![*uncertainty],
    };
    let intrusion = match difference_for_check(
        silk,
        &allowed,
        "silkscreen-to-board-edge-clearance",
        vec![silk_name.to_string(), board_name.to_string()],
    ) {
        Ok(intrusion) => intrusion,
        Err(uncertainty) => return vec![*uncertainty],
    };
    shapes_violation_scalar(
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
    mask: &PcbRegion,
    board_name: &str,
    board: &PcbRegion,
    clearance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let allowed = match offset_for_check(
        board,
        -clearance.clone(),
        "solder-mask-to-board-edge-clearance",
        vec![mask_name.to_string(), board_name.to_string()],
    ) {
        Ok(allowed) => allowed,
        Err(uncertainty) => return vec![*uncertainty],
    };
    let intrusion = match difference_for_check(
        mask,
        &allowed,
        "solder-mask-to-board-edge-clearance",
        vec![mask_name.to_string(), board_name.to_string()],
    ) {
        Ok(intrusion) => intrusion,
        Err(uncertainty) => return vec![*uncertainty],
    };
    shapes_violation_scalar(
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
///
/// Paste apertures are differenced against only nearby copper candidates before
/// exact CSG subtraction. Conservative spatial partitioning keeps sparse
/// copper/paste layers bounded without
/// reporting bbox-only approximations.
pub fn paste_overhang(
    paste_name: &str,
    paste: &PcbRegion,
    copper_name: &str,
    copper: &PcbRegion,
    tolerance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let overhang = if crate::scalar::eq(tolerance, &Scalar::zero()) {
        // Split the authoritative retained regions into exact material
        // components and subtract only exact-AABB candidates. This avoids one
        // all-to-all Boolean without projecting a topology decision to f64.
        exact_componentwise_difference(
            "paste-aperture-overhang",
            paste_name,
            paste,
            copper_name,
            copper,
            None,
        )
        .unwrap_or_else(|| {
            difference_for_check(
                paste,
                copper,
                "paste-aperture-overhang",
                vec![paste_name.to_string(), copper_name.to_string()],
            )
        })
    } else {
        exact_componentwise_difference(
            "paste-aperture-overhang",
            paste_name,
            paste,
            copper_name,
            copper,
            Some(tolerance),
        )
        .unwrap_or_else(|| {
            indexed_difference(
                "paste-aperture-overhang",
                paste_name,
                paste,
                copper_name,
                copper,
                scalar_broad_phase_radius(tolerance),
                IndexedDifferenceMode::CoverOffset(tolerance.clone()),
            )
        })
    };
    let overhang = match overhang {
        Ok(overhang) => overhang,
        Err(uncertainty) => return vec![*uncertainty],
    };
    shapes_violation(
        "paste-aperture-overhang",
        Severity::Warning,
        vec![paste_name.to_string(), copper_name.to_string()],
        overhang,
        min_area,
        format!("paste extends outside copper expanded by tolerance {tolerance}"),
    )
}

/// Run paste overhang against retained per-feature copper geometry.
///
/// Native callers that retain authored copper should prefer this form over an
/// overlapping aggregate image. Candidate pruning remains conservative and
/// every retained subtraction uses exact HyperCurve topology.
pub fn paste_overhang_from_features(
    paste_name: &str,
    paste: &PcbRegion,
    copper_name: &str,
    aggregate_copper: &PcbRegion,
    copper_features: &[&PcbRegion],
    tolerance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let Some(subjects) = exact_region_components(paste) else {
        return paste_overhang(
            paste_name,
            paste,
            copper_name,
            aggregate_copper,
            tolerance,
            min_area,
        );
    };
    let mut covers = Vec::new();
    for feature in copper_features {
        let Some(mut components) = exact_region_components_or_whole(feature) else {
            return paste_overhang(
                paste_name,
                paste,
                copper_name,
                aggregate_copper,
                tolerance,
                min_area,
            );
        };
        covers.append(&mut components);
    }
    let cover_offset = crate::scalar::ne(tolerance, &Scalar::zero()).then_some(tolerance);
    let overhang = match exact_componentwise_difference_from_components(
        "paste-aperture-overhang",
        paste_name,
        paste.metadata(),
        subjects,
        copper_name,
        covers,
        cover_offset,
    ) {
        Some(Ok(overhang)) => overhang,
        Some(Err(uncertainty)) => return vec![*uncertainty],
        None => {
            return paste_overhang(
                paste_name,
                paste,
                copper_name,
                aggregate_copper,
                tolerance,
                min_area,
            );
        }
    };
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
///
/// Coverage is checked per copper island against indexed nearby paste apertures.
/// The private layer-polygon index is a broad phase; exact CSG subtraction decides
/// whether uncovered copper exists.
pub fn paste_aperture_coverage(
    paste_name: &str,
    paste: &PcbRegion,
    copper_name: &str,
    copper: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    let uncovered_copper = match indexed_difference(
        "paste-aperture-coverage",
        copper_name,
        copper,
        paste_name,
        paste,
        0.0,
        IndexedDifferenceMode::CoverAsIs,
    ) {
        Ok(uncovered) => uncovered,
        Err(uncertainty) => return vec![*uncertainty],
    };
    shapes_violation_scalar(
        "paste-aperture-coverage",
        Severity::Warning,
        vec![paste_name.to_string(), copper_name.to_string()],
        uncovered_copper,
        min_area,
        "copper is not covered by a paste aperture".to_string(),
    )
}

/// Run the `solder_mask_overlap_clearance` design-readiness check or report helper.
///
/// Mask opening clearance bands are built from indexed nearby openings before
/// exact intersection with copper. Bounding-box candidates limit CSG workload;
/// offset-ring intersection decides the warning geometry.
pub fn solder_mask_overlap_clearance(
    copper_name: &str,
    copper: &PcbRegion,
    mask_name: &str,
    mask: &PcbRegion,
    clearance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let vulnerable_copper = match indexed_intersection_with_mode(
        "solder-mask-overlap-clearance",
        copper_name,
        copper,
        mask_name,
        mask,
        scalar_broad_phase_radius(clearance),
        IndexedCoverMode::OffsetRing(clearance.clone()),
    ) {
        Ok(vulnerable) => vulnerable,
        Err(uncertainty) => return vec![*uncertainty],
    };
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
///
/// Paste candidates for each copper island are selected with the private
/// layer-polygon spatial index before exact copper/paste intersection, keeping sparse stencil
/// layers from scanning every aperture for every pad.
pub fn paste_aperture_ratio(
    paste_name: &str,
    paste: &PcbRegion,
    copper_name: &str,
    copper: &PcbRegion,
    min_ratio: &Scalar,
    max_ratio: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let paste_polygons = paste.to_multipolygon().0;
    let paste_index = LayerPolygonSpatialIndex::new(&paste_polygons, 0.0);
    log::trace!(
        "paste-aperture-ratio: paste_layer={paste_name} copper_layer={copper_name} paste_apertures={} paste_buckets={} min_ratio={min_ratio:#.3} max_ratio={max_ratio:#.3}",
        paste_polygons.len(),
        paste_index.bucket_count()
    );
    let mut candidate_apertures = 0usize;

    for (island_index, copper_polygon) in copper.to_multipolygon().0.into_iter().enumerate() {
        let Some(copper_area) = polygon_area_scalar(&copper_polygon) else {
            continue;
        };
        if crate::scalar::le(&copper_area, min_area) {
            continue;
        }

        let island = polygon_to_profile(copper_polygon.clone(), Some(metadata(copper_name)));
        let candidate_indexes = paste_index.candidates_near_polygon(&copper_polygon, 0.0);
        candidate_apertures += candidate_indexes.len();
        let mut qualifying_areas = Vec::new();
        for index in candidate_indexes {
            let paste_polygon = &paste_polygons[index];
            let paste_island =
                polygon_to_profile(paste_polygon.clone(), Some(metadata(paste_name)));
            let overlap = match intersection_for_check(
                &island,
                &paste_island,
                "paste-aperture-ratio",
                vec![paste_name.to_string(), copper_name.to_string()],
            ) {
                Ok(overlap) => overlap,
                Err(uncertainty) => return vec![*uncertainty],
            };
            if multipolygon_area_scalar(&overlap.to_multipolygon())
                .is_some_and(|area| crate::scalar::gt(&area, min_area))
                && let Some(area) = polygon_area_scalar(paste_polygon)
            {
                qualifying_areas.push(area);
            }
        }
        let paste_area = Scalar::sum_owned(qualifying_areas);
        let Ok(ratio) = paste_area / copper_area else {
            continue;
        };

        if crate::scalar::ge(&ratio, min_ratio) && crate::scalar::le(&ratio, max_ratio) {
            continue;
        }

        violations.push(Violation::new(
            "paste-aperture-ratio",
            Severity::Warning,
            vec![paste_name.to_string(), copper_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes_scalar(&island.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "paste-to-copper area ratio {ratio:#.3} is outside configured range {min_ratio:#.3}..{max_ratio:#.3}"
            )),
        ));
    }

    log::trace!(
        "paste-aperture-ratio finished: paste_layer={paste_name} copper_layer={copper_name} candidate_apertures={} violations={}",
        candidate_apertures,
        violations.len()
    );

    violations
}

/// Run the `minimum_paste_aperture` design-readiness check or report helper.
pub fn minimum_paste_aperture(
    paste_name: &str,
    paste: &PcbRegion,
    min_width: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for (island_index, polygon) in paste.to_multipolygon().0.into_iter().enumerate() {
        let Some(bounds) = polygon_bounds_scalar(&polygon) else {
            continue;
        };
        let width = &bounds[2] - &bounds[0];
        let height = &bounds[3] - &bounds[1];
        let smallest_dimension = if crate::scalar::le(&width, &height) {
            width
        } else {
            height
        };
        let Some(area) = polygon_area_scalar(&polygon) else {
            continue;
        };

        if crate::scalar::ge(&smallest_dimension, min_width) || crate::scalar::le(&area, min_area) {
            continue;
        }

        let aperture = polygon_to_profile(polygon, Some(metadata(paste_name)));
        violations.push(Violation::new(
            "minimum-paste-aperture",
            Severity::Warning,
            vec![paste_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes_scalar(&aperture.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "paste aperture minimum dimension {smallest_dimension:#.6} is below {min_width:#.6}"
            )),
        ));
    }

    violations
}

/// Run the `paste_aperture_spacing` design-readiness check or report helper.
///
/// Candidate aperture neighbors are selected through `LayerPolygonSpatialIndex`
/// before exact offset/intersection review. The index is only a conservative
/// broad phase, so
/// sparse paste layers avoid repeated whole-layer CSG unions without changing
/// the exact violation predicate.
pub fn paste_aperture_spacing(
    paste_name: &str,
    paste: &PcbRegion,
    min_spacing: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let polygons = paste.to_multipolygon().0;
    let broad_phase_spacing = scalar_broad_phase_radius(min_spacing);
    let index = LayerPolygonSpatialIndex::new(&polygons, broad_phase_spacing);
    log::trace!(
        "paste-aperture-spacing: layer={paste_name} apertures={} buckets={} min_spacing={min_spacing:.6}",
        polygons.len(),
        index.bucket_count()
    );
    let mut violations = Vec::new();
    let expansion = crate::scalar::half(min_spacing);
    let mut candidate_pairs = 0usize;

    for island_index in 0..polygons.len() {
        let candidate_indexes = index.candidates_near(island_index, broad_phase_spacing);
        candidate_pairs += candidate_indexes.len();
        if candidate_indexes.is_empty() {
            continue;
        }

        let island = polygon_to_profile(polygons[island_index].clone(), Some(metadata(paste_name)));
        let candidate_polygons = candidate_indexes
            .into_iter()
            .map(|index| polygons[index].clone())
            .collect::<Vec<_>>();
        let remaining = polygons_to_profile(candidate_polygons, Some(metadata(paste_name)));
        let expanded_island = match offset_for_check(
            &island,
            expansion.clone(),
            "paste-aperture-spacing",
            vec![paste_name.to_string()],
        ) {
            Ok(expanded) => expanded,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let expanded_remaining = match offset_for_check(
            &remaining,
            expansion.clone(),
            "paste-aperture-spacing",
            vec![paste_name.to_string()],
        ) {
            Ok(expanded) => expanded,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let overlap = match intersection_for_check(
            &expanded_island,
            &expanded_remaining,
            "paste-aperture-spacing",
            vec![paste_name.to_string()],
        ) {
            Ok(overlap) => overlap,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
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

    log::trace!(
        "paste-aperture-spacing finished: layer={paste_name} candidate_pairs={} violations={}",
        candidate_pairs,
        violations.len()
    );

    violations
}

/// Run the `paste_mask_alignment` design-readiness check or report helper.
///
/// Paste islands are differenced against only indexed nearby solder-mask
/// openings before exact CSG subtraction. Bounding-box proximity is never a
/// finding by itself.
pub fn paste_mask_alignment(
    paste_name: &str,
    paste: &PcbRegion,
    mask_name: &str,
    mask: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    let outside_mask_opening = match indexed_difference(
        "paste-mask-alignment",
        paste_name,
        paste,
        mask_name,
        mask,
        0.0,
        IndexedDifferenceMode::CoverAsIs,
    ) {
        Ok(outside) => outside,
        Err(uncertainty) => return vec![*uncertainty],
    };
    shapes_violation_scalar(
        "paste-mask-alignment",
        Severity::Warning,
        vec![paste_name.to_string(), mask_name.to_string()],
        outside_mask_opening,
        min_area,
        "paste aperture extends outside the paired solder mask opening".to_string(),
    )
}

/// Run the `exposed_copper` design-readiness check or report helper.
///
/// Copper islands are intersected only with indexed nearby mask openings before
/// exact CSG. The index only bounds
/// candidate generation, while polygon intersection decides the finding.
pub fn exposed_copper(
    copper_name: &str,
    copper: &PcbRegion,
    mask_name: &str,
    mask_openings: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    let overlap = match indexed_intersection(
        "exposed-copper",
        copper_name,
        copper,
        mask_name,
        mask_openings,
        0.0,
    ) {
        Ok(overlap) => overlap,
        Err(uncertainty) => return vec![*uncertainty],
    };
    shapes_violation_scalar(
        "exposed-copper",
        Severity::Warning,
        vec![copper_name.to_string(), mask_name.to_string()],
        overlap,
        min_area,
        "copper intersects solder mask openings".to_string(),
    )
}

/// Run the `solder_mask_opening_coverage` design-readiness check or report helper.
///
/// Copper islands are differenced against indexed nearby solder-mask openings,
/// then exact CSG decides the uncovered shape. This avoids whole-layer boolean
/// work on sparse mask-opening exports while preserving the existing rule
/// semantics.
pub fn solder_mask_opening_coverage(
    copper_name: &str,
    copper: &PcbRegion,
    mask_name: &str,
    mask_openings: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    let covered_copper = match indexed_difference(
        "solder-mask-opening-coverage",
        copper_name,
        copper,
        mask_name,
        mask_openings,
        0.0,
        IndexedDifferenceMode::CoverAsIs,
    ) {
        Ok(covered) => covered,
        Err(uncertainty) => return vec![*uncertainty],
    };
    shapes_violation_scalar(
        "solder-mask-opening-coverage",
        Severity::Error,
        vec![copper_name.to_string(), mask_name.to_string()],
        covered_copper,
        min_area,
        "copper is not covered by a solder mask opening".to_string(),
    )
}

/// Warn when paired solder-mask openings are unusually small or large for copper.
///
/// This is a flattened-layer proxy for NSMD/SMD and BGA mask-opening review:
/// each copper island is matched to indexed nearby mask openings, exact
/// copper/mask intersection confirms the candidate, and the total opening area
/// is compared with the copper island area. Merged openings, under-opened pads,
/// and excessive growth can all change solder joint geometry, so HyperDRC
/// reports the ratio for release review instead of inferring the intended mask
/// definition.
///
/// BGA escape geometry and pad definition choices affect solder-joint
/// performance. Candidate selection is only a broad phase before exact CSG.
pub fn solder_mask_opening_ratio_readiness(
    copper_name: &str,
    copper: &PcbRegion,
    mask_name: &str,
    mask_openings: &PcbRegion,
    min_ratio: &Scalar,
    max_ratio: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let mask_polygons = mask_openings.to_multipolygon().0;
    let mask_index = LayerPolygonSpatialIndex::new(&mask_polygons, 0.0);
    log::trace!(
        "solder-mask opening-ratio readiness: copper_layer={copper_name} mask_layer={mask_name} mask_openings={} mask_buckets={} min_ratio={min_ratio:#.3} max_ratio={max_ratio:#.3}",
        mask_polygons.len(),
        mask_index.bucket_count()
    );
    let mut candidate_openings = 0usize;

    for (island_index, copper_polygon) in copper.to_multipolygon().0.into_iter().enumerate() {
        let Some(copper_area) = polygon_area_scalar(&copper_polygon) else {
            continue;
        };
        if crate::scalar::le(&copper_area, min_area) {
            continue;
        }

        let island = polygon_to_profile(copper_polygon.clone(), Some(metadata(copper_name)));
        let candidate_indexes = mask_index.candidates_near_polygon(&copper_polygon, 0.0);
        candidate_openings += candidate_indexes.len();
        let mut qualifying_areas = Vec::new();
        for index in candidate_indexes {
            let mask_polygon = &mask_polygons[index];
            let mask_island = polygon_to_profile(mask_polygon.clone(), Some(metadata(mask_name)));
            let overlap = match intersection_for_check(
                &island,
                &mask_island,
                "solder-mask-opening-ratio-readiness",
                vec![copper_name.to_string(), mask_name.to_string()],
            ) {
                Ok(overlap) => overlap,
                Err(uncertainty) => return vec![*uncertainty],
            };
            if multipolygon_area_scalar(&overlap.to_multipolygon())
                .is_some_and(|area| crate::scalar::gt(&area, min_area))
                && let Some(area) = polygon_area_scalar(mask_polygon)
            {
                qualifying_areas.push(area);
            }
        }
        let opening_area = Scalar::sum_owned(qualifying_areas);
        let Ok(ratio) = opening_area / copper_area else {
            continue;
        };

        if crate::scalar::ge(&ratio, min_ratio) && crate::scalar::le(&ratio, max_ratio) {
            continue;
        }

        violations.push(Violation::new(
            "solder-mask-opening-ratio-readiness",
            Severity::Warning,
            vec![copper_name.to_string(), mask_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes_scalar(&island.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "solder-mask opening-to-copper area ratio {ratio:#.3} is outside configured range {min_ratio:#.3}..{max_ratio:#.3}; review NSMD/SMD pad definition and BGA mask opening growth"
            )),
        ));
    }

    log::trace!(
        "solder-mask opening-ratio readiness finished: copper_layer={copper_name} mask_layer={mask_name} candidate_openings={} violations={}",
        candidate_openings,
        violations.len()
    );

    violations
}

/// Warn when solder-mask openings do not provide minimum relief around copper.
///
/// This complements [`solder_mask_opening_coverage`]: coverage catches
/// mask-on-pad, while this check asks whether the opening still covers the pad
/// after expanding copper by the configured mask annular ring. It is a
/// manufacturability proxy for mask registration tolerance and avoids claiming
/// exact soldermask-process capability from Gerber polygons alone.
///
/// Per-island lookup only proposes candidates. Lateral process variation for
/// fine features near manufacturing limits must be budgeted before release.
pub fn solder_mask_annular_ring_readiness(
    copper_name: &str,
    copper: &PcbRegion,
    mask_name: &str,
    mask_openings: &PcbRegion,
    min_mask_annular_ring: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    if crate::scalar::le(min_mask_annular_ring, &Scalar::zero()) {
        return Vec::new();
    }

    let copper_polygons = copper.to_multipolygon().0;
    let copper_count = copper_polygons.len();
    let mask_polygons = mask_openings.to_multipolygon().0;
    let broad_phase_ring = scalar_broad_phase_radius(min_mask_annular_ring);
    let mask_index = LayerPolygonSpatialIndex::new(&mask_polygons, broad_phase_ring);
    log::trace!(
        "solder-mask annular-ring readiness: copper={copper_name} copper_islands={} mask={mask_name} mask_openings={} mask_buckets={} min_ring={min_mask_annular_ring:.6}",
        copper_count,
        mask_polygons.len(),
        mask_index.bucket_count()
    );

    let mut violations = Vec::new();
    let mut candidate_openings = 0usize;

    for (island_index, copper_polygon) in copper_polygons.into_iter().enumerate() {
        if polygon_area_scalar(&copper_polygon)
            .is_none_or(|area| crate::scalar::le(&area, min_area))
        {
            continue;
        }

        let candidates = mask_index.candidates_near_polygon(&copper_polygon, broad_phase_ring);
        candidate_openings += candidates.len();
        let copper_island = polygon_to_profile(copper_polygon, Some(metadata(copper_name)));
        let required_opening = match offset_for_check(
            &copper_island,
            min_mask_annular_ring.clone(),
            "solder-mask-annular-ring-readiness",
            vec![copper_name.to_string(), mask_name.to_string()],
        ) {
            Ok(required_opening) => required_opening,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let missing_relief = if candidates.is_empty() {
            required_opening
        } else {
            let candidate_openings = candidates
                .into_iter()
                .map(|index| mask_polygons[index].clone())
                .collect::<Vec<_>>();
            let mask_region = polygons_to_profile(candidate_openings, Some(metadata(mask_name)));
            match difference_for_check(
                &required_opening,
                &mask_region,
                "solder-mask-annular-ring-readiness",
                vec![copper_name.to_string(), mask_name.to_string()],
            ) {
                Ok(missing_relief) => missing_relief,
                Err(uncertainty) => return vec![*uncertainty],
            }
        };
        let shapes = multipolygon_to_shapes_scalar(&missing_relief.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "solder-mask-annular-ring-readiness",
            Severity::Warning,
            vec![copper_name.to_string(), mask_name.to_string()],
            Some(island_index),
            shapes,
            Vec::new(),
            Some(format!(
                "solder mask opening does not cover copper expanded by minimum mask annular ring {min_mask_annular_ring:.6}; review mask registration and opening growth"
            )),
        ));
    }

    log::trace!(
        "solder-mask annular-ring readiness finished: copper={copper_name} mask={mask_name} candidate_openings={} violations={}",
        candidate_openings,
        violations.len()
    );

    violations
}

/// Run the `solder_mask_expansion` design-readiness check or report helper.
///
/// Mask openings are compared to indexed nearby copper, expanded only for exact
/// candidate subtraction. The conservative index lets large sparse fields avoid
/// global copper offsets.
pub fn solder_mask_expansion(
    copper_name: &str,
    copper: &PcbRegion,
    mask_name: &str,
    mask_openings: &PcbRegion,
    max_expansion: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let excessive_opening = match exact_componentwise_difference(
        "solder-mask-expansion",
        mask_name,
        mask_openings,
        copper_name,
        copper,
        Some(max_expansion),
    )
    .unwrap_or_else(|| {
        indexed_difference(
            "solder-mask-expansion",
            mask_name,
            mask_openings,
            copper_name,
            copper,
            scalar_broad_phase_radius(max_expansion),
            IndexedDifferenceMode::CoverOffset(max_expansion.clone()),
        )
    }) {
        Ok(excessive) => excessive,
        Err(uncertainty) => return vec![*uncertainty],
    };
    shapes_violation(
        "solder-mask-expansion",
        Severity::Warning,
        vec![copper_name.to_string(), mask_name.to_string()],
        excessive_opening,
        min_area,
        format!("solder mask opening exceeds copper expansion {max_expansion}"),
    )
}

/// Run solder-mask expansion against retained per-feature copper geometry.
///
/// Native callers should prefer this form when they retain authored pads,
/// routes, vias, and zones. Small exact land patterns are considered before
/// large planes, avoiding a lossy aggregate decomposition and repeated offsets
/// while producing the same set difference as the aggregate-layer check.
pub fn solder_mask_expansion_from_features(
    copper_name: &str,
    aggregate_copper: &PcbRegion,
    copper_features: &[&PcbRegion],
    mask_name: &str,
    mask_openings: &PcbRegion,
    max_expansion: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let Some(subjects) = exact_region_components(mask_openings) else {
        return solder_mask_expansion(
            copper_name,
            aggregate_copper,
            mask_name,
            mask_openings,
            max_expansion,
            min_area,
        );
    };
    let mut covers = Vec::new();
    for feature in copper_features {
        let Some(mut components) = exact_region_components_or_whole(feature) else {
            return solder_mask_expansion(
                copper_name,
                aggregate_copper,
                mask_name,
                mask_openings,
                max_expansion,
                min_area,
            );
        };
        covers.append(&mut components);
    }
    let excessive_opening = match exact_componentwise_difference_from_components(
        "solder-mask-expansion",
        mask_name,
        mask_openings.metadata(),
        subjects,
        copper_name,
        covers,
        Some(max_expansion),
    ) {
        Some(Ok(excessive)) => excessive,
        Some(Err(uncertainty)) => return vec![*uncertainty],
        None => {
            return solder_mask_expansion(
                copper_name,
                aggregate_copper,
                mask_name,
                mask_openings,
                max_expansion,
                min_area,
            );
        }
    };
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
///
/// Silkscreen geometry is intersected only with indexed nearby blocker
/// candidates before exact CSG, keeping sparse legend/blocker
/// exports bounded without changing the reported geometry predicate.
pub fn silkscreen_overlap(
    silk_name: &str,
    silk: &PcbRegion,
    blocker_name: &str,
    blocker: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    let overlap = match indexed_intersection_with_mode(
        "silkscreen-overlap",
        silk_name,
        silk,
        blocker_name,
        blocker,
        0.0,
        IndexedCoverMode::AsIs,
    ) {
        Ok(overlap) => overlap,
        Err(uncertainty) => return vec![*uncertainty],
    };
    shapes_violation_scalar(
        "silkscreen-overlap",
        Severity::Warning,
        vec![silk_name.to_string(), blocker_name.to_string()],
        overlap,
        min_area,
        "silkscreen overlaps copper or exposed-pad geometry".to_string(),
    )
}

/// Run the `silkscreen_clearance` design-readiness check or report helper.
///
/// Blocker candidates are selected through `LayerPolygonSpatialIndex`, then
/// expanded and intersected exactly. The index is only a broad phase, so clearance findings remain
/// CSG-derived.
pub fn silkscreen_clearance(
    silk_name: &str,
    silk: &PcbRegion,
    blocker_name: &str,
    blocker: &PcbRegion,
    clearance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let intrusion = match indexed_intersection_with_mode(
        "silkscreen-clearance",
        silk_name,
        silk,
        blocker_name,
        blocker,
        scalar_broad_phase_radius(clearance),
        IndexedCoverMode::Offset(clearance.clone()),
    ) {
        Ok(intrusion) => intrusion,
        Err(uncertainty) => return vec![*uncertainty],
    };
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
    silk: &PcbRegion,
    min_width: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let radius = crate::scalar::half(min_width);
    let polygons = silk.to_multipolygon().0;
    log::trace!(
        "silkscreen-min-width: layer={silk_name} polygons={} min_width={min_width:#.6}",
        polygons.len()
    );
    let mut shapes = Vec::new();

    for polygon in polygons {
        // Apply morphological opening to one disconnected legend island at a
        // time. Whole-layer opening can create pathological boolean operations
        // on dense Gerber packages, while island-local opening is equivalent
        // for independent silkscreen strokes.
        let dimension_below_limit = polygon_minimum_bounding_dimension_scalar(&polygon)
            .is_some_and(|dimension| {
                crate::scalar::gt(&dimension, &Scalar::zero())
                    && crate::scalar::lt(&dimension, min_width)
            });
        let area_above_gate =
            polygon_area_scalar(&polygon).is_some_and(|area| crate::scalar::gt(&area, min_area));
        let island = polygon_to_profile(polygon, Some(metadata(silk_name)));
        let eroded = match offset_for_check(
            &island,
            -radius.clone(),
            "silkscreen-min-width",
            vec![silk_name.to_string()],
        ) {
            Ok(eroded) => eroded,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let reconstructed = match offset_for_check(
            &eroded,
            radius.clone(),
            "silkscreen-min-width",
            vec![silk_name.to_string()],
        ) {
            Ok(reconstructed) => reconstructed,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let thin_features = match difference_for_check(
            &island,
            &reconstructed,
            "silkscreen-min-width",
            vec![silk_name.to_string()],
        ) {
            Ok(thin_features) => thin_features,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let mut island_shapes =
            multipolygon_to_shapes_scalar(&thin_features.to_multipolygon(), min_area);
        // Native offset can conservatively retain an island when collapse is
        // uncertain. The exact promoted envelope remains a sound readiness
        // fallback for a globally undersized disconnected stroke.
        if island_shapes.is_empty() && dimension_below_limit && area_above_gate {
            island_shapes = multipolygon_to_shapes_scalar(&island.to_multipolygon(), min_area);
        }
        shapes.extend(island_shapes);
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

/// Warn when disconnected silkscreen islands are smaller than a text-height budget.
///
/// Flattened Gerber geometry has already lost true text strings, font metrics,
/// and bottom-side mirroring semantics. This check therefore uses the larger
/// bounding-box dimension of each disconnected island as a conservative
/// readability proxy for tiny glyphs, pin-1 dots, polarity marks, and small
/// reference-designator fragments.
///
/// Small physical character size is an explicit release-review parameter rather
/// than an implicit artifact of the CAD font.
pub fn silkscreen_text_height_readiness(
    silk_name: &str,
    silk: &PcbRegion,
    min_text_height: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    if crate::scalar::le(min_text_height, &Scalar::zero()) {
        return Vec::new();
    }

    let polygons = silk.to_multipolygon().0;
    log::trace!(
        "silkscreen text-height readiness: layer={silk_name} islands={} min_text_height={min_text_height:#.6}",
        polygons.len()
    );
    let mut violations = Vec::new();
    let mut measured_islands = 0usize;

    for (island_index, polygon) in polygons.into_iter().enumerate() {
        let Some(area) = polygon_area_scalar(&polygon) else {
            continue;
        };
        if crate::scalar::le(&area, min_area) {
            continue;
        }
        let Some(exact_bounds) = polygon_bounds_scalar(&polygon) else {
            continue;
        };
        let Some(bounds) = polygon.bounding_rect() else {
            continue;
        };
        measured_islands += 1;
        let width = &exact_bounds[2] - &exact_bounds[0];
        let height = &exact_bounds[3] - &exact_bounds[1];
        let apparent_height = if crate::scalar::ge(&width, &height) {
            width
        } else {
            height
        };
        if crate::scalar::ge(&apparent_height, min_text_height) {
            continue;
        }

        let island = polygon_to_profile(polygon, Some(metadata(silk_name)));
        violations.push(Violation::new(
            "silkscreen-text-height-readiness",
            Severity::Warning,
            vec![silk_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes_scalar(&island.to_multipolygon(), min_area),
            vec![[
                (bounds.min().x + bounds.max().x) / 2.0,
                (bounds.min().y + bounds.max().y) / 2.0,
            ]],
            Some(format!(
                "silkscreen island apparent text height {apparent_height:#.6} is below minimum {min_text_height:#.6}; review legend, polarity, and pin-1 mark legibility"
            )),
        ));
    }

    log::trace!(
        "silkscreen text-height readiness finished: layer={silk_name} measured_islands={} violations={}",
        measured_islands,
        violations.len()
    );

    violations
}

/// Run the `min_copper_neck_width` design-readiness check or report helper.
pub fn min_copper_neck_width(
    copper_name: &str,
    copper: &PcbRegion,
    min_width: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    const MAX_MORPHOLOGY_VERTICES: usize = 256;
    let radius = crate::scalar::half(min_width);
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
    let mut complexity_retained_count = 0_usize;
    for (island_index, polygon) in source_polygons.iter().enumerate() {
        log::trace!(
            "min-copper-neck: layer={copper_name} island={island_index} exterior_vertices={} holes={}",
            polygon.exterior().0.len(),
            polygon.interiors().len()
        );
        // A convex island has no local neck narrower than its minimum support
        // width. Certify that width directly with exact dyadic input scalars so
        // ordinary round/rectangular pads do not enter the much more expensive
        // offset-and-Boolean morphology path.
        if convex_polygon_width_at_least(polygon, min_width) {
            continue;
        }
        let area_above_gate =
            polygon_area_scalar(polygon).is_some_and(|area| crate::scalar::gt(&area, min_area));
        if !area_above_gate {
            continue;
        }
        let source = MultiPolygon(vec![polygon.clone()]);
        let dimension_below_limit =
            polygon_minimum_bounding_dimension_scalar(polygon).is_some_and(|dimension| {
                crate::scalar::gt(&dimension, &Scalar::zero())
                    && crate::scalar::lt(&dimension, min_width)
            });
        if dimension_below_limit {
            // Either axis-aligned support width is itself a valid upper bound
            // on global feature width. An island below the rule therefore
            // needs review without an offset or Boolean construction.
            shapes.extend(multipolygon_to_shapes_scalar(&source, min_area));
            continue;
        }
        let morphology_vertices = polygon.exterior().0.len()
            + polygon
                .interiors()
                .iter()
                .map(|ring| ring.0.len())
                .sum::<usize>();
        if morphology_vertices > MAX_MORPHOLOGY_VERTICES {
            // Readiness checks must have a deterministic work bound even for
            // production pours with enormous merged contours. Retaining the
            // candidate is conservative and avoids hiding a possible neck.
            complexity_retained_count += 1;
            shapes.extend(multipolygon_to_shapes_scalar(&source, min_area));
            continue;
        }
        let island = polygon_to_profile(polygon.clone(), Some(metadata(copper_name)));
        let eroded = match offset_for_check(
            &island,
            -radius.clone(),
            "minimum-copper-neck-width",
            vec![copper_name.to_string()],
        ) {
            Ok(eroded) => eroded,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let reconstructed = match offset_for_check(
            &eroded,
            radius.clone(),
            "minimum-copper-neck-width",
            vec![copper_name.to_string()],
        ) {
            Ok(reconstructed) => reconstructed,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let thin = match difference_for_check(
            &island,
            &reconstructed,
            "minimum-copper-neck-width",
            vec![copper_name.to_string()],
        ) {
            Ok(thin_features) => thin_features.to_multipolygon(),
            Err(uncertainty) => return vec![*uncertainty],
        };
        if whole_feature_removal_is_width_compliant(&source, &thin, min_width) {
            continue;
        }
        shapes.extend(multipolygon_to_shapes_scalar(&thin, min_area));
    }

    if shapes.is_empty() && complexity_retained_count == 0 {
        return Vec::new();
    }

    let message = if complexity_retained_count == 0 {
        format!("copper features are removed by opening with width {min_width}")
    } else {
        format!(
            "copper features are removed by opening with width {min_width}; {complexity_retained_count} island(s) were retained because their contours exceeded the bounded morphology complexity"
        )
    };

    vec![Violation::new(
        "minimum-copper-neck-width",
        Severity::Warning,
        vec![copper_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some(message),
    )]
}

fn whole_feature_removal_is_width_compliant(
    source: &MultiPolygon<f64>,
    removed: &MultiPolygon<f64>,
    min_width: &Scalar,
) -> bool {
    let Some(source_area) = multipolygon_area_scalar(source) else {
        return false;
    };
    let Some(removed_area) = multipolygon_area_scalar(removed) else {
        return false;
    };
    if crate::scalar::eq(&source_area, &Scalar::zero())
        || crate::scalar::gt(
            &(&removed_area - &source_area).abs(),
            &(&source_area * crate::scalar::scalar("1.0e-6")),
        )
    {
        return false;
    }

    source.0.iter().all(|polygon| {
        shortest_exterior_segment_scalar(polygon)
            .is_some_and(|length| crate::scalar::ge(&length, min_width))
    })
}

fn shortest_exterior_segment_scalar(polygon: &Polygon<f64>) -> Option<Scalar> {
    let mut lengths = polygon
        .exterior()
        .0
        .windows(2)
        .filter_map(|segment| {
            let dx = Scalar::try_from(segment[1].x).ok()? - Scalar::try_from(segment[0].x).ok()?;
            let dy = Scalar::try_from(segment[1].y).ok()? - Scalar::try_from(segment[0].y).ok()?;
            (&dx * &dx + &dy * &dy).sqrt().ok()
        })
        .filter(|length| crate::scalar::gt(length, &Scalar::zero()));
    let first = lengths.next()?;
    Some(lengths.fold(first, |shortest, length| {
        if crate::scalar::lt(&length, &shortest) {
            length
        } else {
            shortest
        }
    }))
}

/// Run the `solder_mask_sliver` design-readiness check or report helper.
pub fn solder_mask_sliver(
    mask_name: &str,
    mask: &PcbRegion,
    min_width: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let radius = crate::scalar::half(min_width);
    // Same opening operation as the copper neck-width check, applied to residual
    // mask geometry. The result is the geometry that is too thin to survive the
    // configured web width.
    let eroded = match offset_for_check(
        mask,
        -radius.clone(),
        "solder-mask-sliver",
        vec![mask_name.to_string()],
    ) {
        Ok(eroded) => eroded,
        Err(uncertainty) => return vec![*uncertainty],
    };
    let reconstructed = match offset_for_check(
        &eroded,
        radius,
        "solder-mask-sliver",
        vec![mask_name.to_string()],
    ) {
        Ok(reconstructed) => reconstructed,
        Err(uncertainty) => return vec![*uncertainty],
    };
    let slivers = match difference_for_check(
        mask,
        &reconstructed,
        "solder-mask-sliver",
        vec![mask_name.to_string()],
    ) {
        Ok(slivers) => slivers,
        Err(uncertainty) => return vec![*uncertainty],
    };
    let mut violations = shapes_violation_scalar(
        "solder-mask-sliver",
        Severity::Warning,
        vec![mask_name.to_string()],
        slivers,
        min_area,
        format!("solder mask geometry is removed by opening with width {min_width}"),
    );
    if violations.is_empty() {
        let narrow = MultiPolygon(
            mask.to_multipolygon()
                .0
                .into_iter()
                .filter(|polygon| {
                    polygon_minimum_bounding_dimension_scalar(polygon).is_some_and(|dimension| {
                        crate::scalar::gt(&dimension, &Scalar::zero())
                            && crate::scalar::lt(&dimension, min_width)
                    }) && polygon_area_scalar(polygon)
                        .is_some_and(|area| crate::scalar::gt(&area, min_area))
                })
                .collect(),
        );
        let shapes = multipolygon_to_shapes_scalar(&narrow, min_area);
        if !shapes.is_empty() {
            violations.push(Violation::new(
                "solder-mask-sliver",
                Severity::Warning,
                vec![mask_name.to_string()],
                None,
                shapes,
                Vec::new(),
                Some(format!(
                    "solder mask geometry is below configured width {min_width}"
                )),
            ));
        }
    }
    violations
}

/// Run the `minimum_mask_opening` design-readiness check or report helper.
pub fn minimum_mask_opening(
    mask_name: &str,
    mask: &PcbRegion,
    min_opening: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for (island_index, polygon) in mask.to_multipolygon().0.into_iter().enumerate() {
        let Some(bounds) = polygon_bounds_scalar(&polygon) else {
            continue;
        };
        let width = &bounds[2] - &bounds[0];
        let height = &bounds[3] - &bounds[1];
        let smallest_dimension = if crate::scalar::le(&width, &height) {
            width
        } else {
            height
        };
        let Some(area) = polygon_area_scalar(&polygon) else {
            continue;
        };

        if crate::scalar::ge(&smallest_dimension, min_opening) || crate::scalar::le(&area, min_area)
        {
            continue;
        }

        let opening = polygon_to_profile(polygon, Some(metadata(mask_name)));
        violations.push(Violation::new(
            "minimum-mask-opening",
            Severity::Warning,
            vec![mask_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes_scalar(&opening.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "solder mask opening minimum dimension {smallest_dimension:#.6} is below {min_opening:#.6}"
            )),
        ));
    }

    violations
}

/// Run the `solder_mask_opening_spacing` design-readiness check or report helper.
///
/// Opening-neighbor selection uses the same private layer-polygon spatial index
/// as paste spacing before the exact expanded-opening intersection. This keeps
/// sparse mask-opening layers bounded while retaining the CSG predicate that
/// reports actual solder-mask bridge conflicts.
pub fn solder_mask_opening_spacing(
    mask_name: &str,
    mask: &PcbRegion,
    min_spacing: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let openings = mask.to_multipolygon().0;
    let broad_phase_spacing = scalar_broad_phase_radius(min_spacing);
    let index = LayerPolygonSpatialIndex::new(&openings, broad_phase_spacing);
    log::trace!(
        "solder-mask-opening-spacing: layer={mask_name} openings={} buckets={} min_spacing={min_spacing:.6}",
        openings.len(),
        index.bucket_count()
    );
    let mut violations = Vec::new();
    let expansion = crate::scalar::half(min_spacing);
    let mut candidate_pairs = 0usize;

    for opening_index in 0..openings.len() {
        let candidate_indexes = index.later_candidates_near(opening_index, broad_phase_spacing);
        candidate_pairs += candidate_indexes.len();
        for neighbor_index in candidate_indexes {
            if !polygons_within_clearance(
                &openings[opening_index],
                &openings[neighbor_index],
                broad_phase_spacing,
            ) {
                continue;
            }

            let opening =
                polygon_to_profile(openings[opening_index].clone(), Some(metadata(mask_name)));
            let neighbor =
                polygon_to_profile(openings[neighbor_index].clone(), Some(metadata(mask_name)));
            let expanded_opening = match offset_for_check(
                &opening,
                expansion.clone(),
                "solder-mask-opening-spacing",
                vec![mask_name.to_string()],
            ) {
                Ok(expanded) => expanded,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let expanded_neighbor = match offset_for_check(
                &neighbor,
                expansion.clone(),
                "solder-mask-opening-spacing",
                vec![mask_name.to_string()],
            ) {
                Ok(expanded) => expanded,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let bridge_conflict = match intersection_for_check(
                &expanded_opening,
                &expanded_neighbor,
                "solder-mask-opening-spacing",
                vec![mask_name.to_string()],
            ) {
                Ok(bridge_conflict) => bridge_conflict,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let shapes =
                multipolygon_to_shapes_scalar(&bridge_conflict.to_multipolygon(), min_area);
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

    log::trace!(
        "solder-mask-opening-spacing finished: layer={mask_name} candidate_pairs={} violations={}",
        candidate_pairs,
        violations.len()
    );

    violations
}

/// Run the `acid_trap_candidates` design-readiness check or report helper.
pub fn acid_trap_candidates(
    copper_name: &str,
    copper: &PcbRegion,
    max_angle_degrees: &Scalar,
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
            "copper polygon vertices below {max_angle_degrees:#.3} degrees"
        )),
    )]
}

/// Run the `layer_sanity` design-readiness check or report helper.
pub fn layer_sanity(
    layer_name: &str,
    region: &PcbRegion,
    max_layer_area: Option<&Scalar>,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let multipolygon = region.to_multipolygon();
    let area = balanced_scalar_sum(
        multipolygon
            .0
            .iter()
            .map(polygon_area_scalar)
            .collect::<Option<Vec<_>>>()
            .unwrap_or_default(),
    );

    if let Some(detail) = region.exact_construction_error() {
        violations.push(Violation::new(
            "layer-sanity",
            Severity::Error,
            vec![layer_name.to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "layer exact geometry could not be constructed and must not be treated as empty: {detail}"
            )),
        ));
    }

    if region.had_non_finite_input() || multipolygon_has_non_finite_coordinates(&multipolygon) {
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

    if matches!(
        compare_reals_with_policy(&area, &Scalar::zero(), PredicatePolicy).value(),
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
    ) {
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
        && compare_reals_with_policy(&area, max_layer_area, PredicatePolicy).value()
            == Some(std::cmp::Ordering::Greater)
    {
        let shapes = multipolygon_to_shapes_scalar(&multipolygon, &Scalar::zero());
        violations.push(Violation::new(
            "layer-sanity",
            Severity::Warning,
            vec![layer_name.to_string()],
            None,
            shapes,
            Vec::new(),
            Some(format!(
                "layer area {area:#.9} exceeds maximum expected area {max_layer_area:#.9}"
            )),
        ));
    }

    if region.geometry().bounding_rect().is_none() {
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
/// to the process. HyperDRC uses tiny features as a review trigger rather than a
/// claim that the specific island cannot be fabricated.
pub fn tiny_layer_feature_readiness(
    layer_name: &str,
    region: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    if crate::scalar::le(min_area, &Scalar::zero()) {
        return Vec::new();
    }

    let multipolygon = region.to_multipolygon();
    let tiny_polygons = multipolygon
        .0
        .iter()
        .filter(|polygon| {
            polygon_area_scalar(polygon).is_some_and(|area| {
                compare_reals_with_policy(&area, &Scalar::zero(), PredicatePolicy).value()
                    == Some(std::cmp::Ordering::Greater)
                    && matches!(
                        compare_reals_with_policy(&area, min_area, PredicatePolicy).value(),
                        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                    )
            })
        })
        .cloned()
        .collect::<Vec<_>>();
    log::trace!(
        "tiny-layer feature readiness: layer={layer_name} polygons={} tiny_polygons={} min_area={min_area:#.9}",
        multipolygon.0.len(),
        tiny_polygons.len()
    );

    if tiny_polygons.is_empty() {
        return Vec::new();
    }

    let tiny = MultiPolygon(tiny_polygons);
    let locations = duplicate_layer_locations(&tiny);
    let shapes = multipolygon_to_shapes_scalar(&tiny, &Scalar::zero());
    vec![Violation::new(
        "layer-sanity",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        shapes,
        locations,
        Some(format!(
            "layer contains polygon islands at or below reportable feature area {min_area:#.9}; review for tiny aperture flashes, fractured slivers, or stale artifacts"
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
    region: &PcbRegion,
    min_width: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    if crate::scalar::le(min_width, &Scalar::zero()) {
        return Vec::new();
    }

    let multipolygon = region.to_multipolygon();
    let skinny_polygons = multipolygon
        .0
        .iter()
        .filter(|polygon| {
            polygon_area_scalar(polygon).is_some_and(|area| {
                compare_reals_with_policy(&area, min_area, PredicatePolicy).value()
                    == Some(std::cmp::Ordering::Greater)
            }) && polygon_minimum_bounding_dimension_scalar(polygon).is_some_and(|dimension| {
                compare_reals_with_policy(&dimension, &Scalar::zero(), PredicatePolicy).value()
                    == Some(std::cmp::Ordering::Greater)
                    && compare_reals_with_policy(&dimension, min_width, PredicatePolicy).value()
                        == Some(std::cmp::Ordering::Less)
            })
        })
        .cloned()
        .collect::<Vec<_>>();
    log::trace!(
        "skinny-layer feature readiness: layer={layer_name} polygons={} skinny_polygons={} min_width={min_width:#.6} min_area={min_area:#.9}",
        multipolygon.0.len(),
        skinny_polygons.len()
    );

    if skinny_polygons.is_empty() {
        return Vec::new();
    }

    let skinny = MultiPolygon(skinny_polygons);
    let locations = duplicate_layer_locations(&skinny);
    let shapes = multipolygon_to_shapes_scalar(&skinny, min_area);
    vec![Violation::new(
        "layer-sanity",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        shapes,
        locations,
        Some(format!(
            "layer contains polygon islands whose minimum bounding dimension is below feature width {min_width:#.6}; review for hairline fragments, slivers, or overdrawn route artifacts"
        )),
    )]
}

fn polygon_minimum_bounding_dimension_scalar(polygon: &Polygon<f64>) -> Option<Scalar> {
    let bounds = polygon_bounds_scalar(polygon)?;
    let width = &bounds[2] - &bounds[0];
    let height = &bounds[3] - &bounds[1];
    Some(if crate::scalar::le(&width, &height) {
        width
    } else {
        height
    })
}

#[derive(Clone, Copy)]
struct OutwardInterval {
    lower: f64,
    upper: f64,
}

impl OutwardInterval {
    fn exact(value: f64) -> Option<Self> {
        value.is_finite().then_some(Self {
            lower: value,
            upper: value,
        })
    }

    fn add(self, other: Self) -> Option<Self> {
        let lower = (self.lower + other.lower).next_down();
        let upper = (self.upper + other.upper).next_up();
        (lower.is_finite() && upper.is_finite()).then_some(Self { lower, upper })
    }

    fn subtract(self, other: Self) -> Option<Self> {
        let lower = (self.lower - other.upper).next_down();
        let upper = (self.upper - other.lower).next_up();
        (lower.is_finite() && upper.is_finite()).then_some(Self { lower, upper })
    }

    fn multiply(self, other: Self) -> Option<Self> {
        let products = [
            self.lower * other.lower,
            self.lower * other.upper,
            self.upper * other.lower,
            self.upper * other.upper,
        ];
        if !products.iter().all(|value| value.is_finite()) {
            return None;
        }
        let lower = products
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
            .next_down();
        let upper = products
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max)
            .next_up();
        Some(Self { lower, upper })
    }

    fn negate(self) -> Self {
        Self {
            lower: -self.upper,
            upper: -self.lower,
        }
    }
}

/// Quickly certify a threshold-diameter disk inside a convex polygon.
///
/// The source coordinates are exactly representable dyadic inputs. Every
/// projected operation below is rounded outward by one representable float, so
/// a `true` result proves the corresponding exact inequalities. Failure merely
/// selects the general exact support-width certificate below.
fn convex_polygon_contains_threshold_disk(polygon: &Polygon<f64>, min_width: &Scalar) -> bool {
    if !polygon.interiors().is_empty() || crate::scalar::le(min_width, &Scalar::zero()) {
        return false;
    }
    let ring = &polygon.exterior().0;
    let vertex_count = if ring.len() >= 2 && ring.first() == ring.last() {
        ring.len() - 1
    } else {
        ring.len()
    };
    if vertex_count < 3
        || !ring[..vertex_count]
            .iter()
            .all(|point| point.x.is_finite() && point.y.is_finite())
    {
        return false;
    }

    // Prove an upper f64 bound for the exact rule width. The lossy projection
    // is only a starting point; lifting it back into Scalar closes the proof.
    let Some(mut width_upper) = min_width.to_f64_lossy() else {
        return false;
    };
    for _ in 0..32 {
        if Scalar::try_from(width_upper)
            .is_ok_and(|projected| crate::scalar::ge(&projected, min_width))
        {
            break;
        }
        width_upper = width_upper.next_up();
    }
    if !width_upper.is_finite() || width_upper <= 0.0 {
        return false;
    }
    let Ok(projected_width_upper) = Scalar::try_from(width_upper) else {
        return false;
    };
    if crate::scalar::lt(&projected_width_upper, min_width) {
        return false;
    }

    let mut min_x = ring[0].x;
    let mut max_x = ring[0].x;
    let mut min_y = ring[0].y;
    let mut max_y = ring[0].y;
    for point in &ring[1..vertex_count] {
        min_x = min_x.min(point.x);
        max_x = max_x.max(point.x);
        min_y = min_y.min(point.y);
        max_y = max_y.max(point.y);
    }
    let Some(half) = OutwardInterval::exact(0.5) else {
        return false;
    };
    let Some(center_x) = OutwardInterval::exact(min_x)
        .and_then(|minimum| minimum.add(OutwardInterval::exact(max_x)?))
        .and_then(|sum| sum.multiply(half))
    else {
        return false;
    };
    let Some(center_y) = OutwardInterval::exact(min_y)
        .and_then(|minimum| minimum.add(OutwardInterval::exact(max_y)?))
        .and_then(|sum| sum.multiply(half))
    else {
        return false;
    };

    let Some(width) = OutwardInterval::exact(width_upper) else {
        return false;
    };
    let Some(width_squared) = width.multiply(width) else {
        return false;
    };
    let Some(four) = OutwardInterval::exact(4.0) else {
        return false;
    };
    let mut turn_direction = None;
    for index in 0..vertex_count {
        let a = ring[index];
        let b = ring[(index + 1) % vertex_count];
        let c = ring[(index + 2) % vertex_count];
        let Some(edge_x) = OutwardInterval::exact(b.x)
            .and_then(|value| value.subtract(OutwardInterval::exact(a.x)?))
        else {
            return false;
        };
        let Some(edge_y) = OutwardInterval::exact(b.y)
            .and_then(|value| value.subtract(OutwardInterval::exact(a.y)?))
        else {
            return false;
        };
        let Some(next_x) = OutwardInterval::exact(c.x)
            .and_then(|value| value.subtract(OutwardInterval::exact(b.x)?))
        else {
            return false;
        };
        let Some(next_y) = OutwardInterval::exact(c.y)
            .and_then(|value| value.subtract(OutwardInterval::exact(b.y)?))
        else {
            return false;
        };
        let Some(turn) = edge_x.multiply(next_y).and_then(|left| {
            edge_y
                .multiply(next_x)
                .and_then(|right| left.subtract(right))
        }) else {
            return false;
        };
        let direction = if turn.lower > 0.0 {
            1_i8
        } else if turn.upper < 0.0 {
            -1_i8
        } else {
            return false;
        };
        if turn_direction.is_some_and(|existing| existing != direction) {
            return false;
        }
        turn_direction = Some(direction);

        let Some(center_dx) = center_x.subtract(OutwardInterval::exact(a.x).unwrap()) else {
            return false;
        };
        let Some(center_dy) = center_y.subtract(OutwardInterval::exact(a.y).unwrap()) else {
            return false;
        };
        let Some(mut center_projection) = edge_x.multiply(center_dy).and_then(|left| {
            edge_y
                .multiply(center_dx)
                .and_then(|right| left.subtract(right))
        }) else {
            return false;
        };
        if direction < 0 {
            center_projection = center_projection.negate();
        }
        if center_projection.lower <= 0.0 {
            return false;
        }

        let Some(edge_length_squared) = edge_x
            .multiply(edge_x)
            .and_then(|x| edge_y.multiply(edge_y).and_then(|y| x.add(y)))
        else {
            return false;
        };
        let Some(left) = center_projection
            .multiply(center_projection)
            .and_then(|squared| four.multiply(squared))
        else {
            return false;
        };
        let Some(right) = width_squared.multiply(edge_length_squared) else {
            return false;
        };
        if left.lower < right.upper {
            return false;
        }
    }
    turn_direction.is_some()
}

/// Certify that a simple convex polygon's minimum support width meets a limit.
///
/// For a convex polygon, a minimum-width enclosing strip has one supporting
/// line collinear with an edge. Exact rotating calipers find the opposite
/// supporting vertex for every edge in linear time, proving the global minimum
/// width without constructing offset contours. Squaring `span / |edge|` keeps
/// the comparison exact and avoids a square root. Polygons with holes,
/// non-convex turns, or malformed edges fall back to the general morphology
/// path.
fn convex_polygon_width_at_least(polygon: &Polygon<f64>, min_width: &Scalar) -> bool {
    if convex_polygon_contains_threshold_disk(polygon, min_width) {
        return true;
    }
    if !polygon.interiors().is_empty() || crate::scalar::le(min_width, &Scalar::zero()) {
        return false;
    }

    let ring = &polygon.exterior().0;
    let vertex_count = if ring.len() >= 2 && ring.first() == ring.last() {
        ring.len() - 1
    } else {
        ring.len()
    };
    if vertex_count < 3 {
        return false;
    }
    let Some(points) = ring[..vertex_count]
        .iter()
        .map(|point| {
            Some((
                Scalar::try_from(point.x).ok()?,
                Scalar::try_from(point.y).ok()?,
            ))
        })
        .collect::<Option<Vec<_>>>()
    else {
        return false;
    };

    let zero = Scalar::zero();
    let mut turn_direction = None;
    for index in 0..vertex_count {
        let a = &points[index];
        let b = &points[(index + 1) % vertex_count];
        let c = &points[(index + 2) % vertex_count];
        let ab_x = &b.0 - &a.0;
        let ab_y = &b.1 - &a.1;
        let bc_x = &c.0 - &b.0;
        let bc_y = &c.1 - &b.1;
        let turn = &ab_x * &bc_y - &ab_y * &bc_x;
        let direction = if crate::scalar::gt(&turn, &zero) {
            1_i8
        } else if crate::scalar::lt(&turn, &zero) {
            -1_i8
        } else {
            continue;
        };
        if turn_direction.is_some_and(|existing| existing != direction) {
            return false;
        }
        turn_direction = Some(direction);
    }
    let Some(turn_direction) = turn_direction else {
        return false;
    };

    let min_width_squared = min_width * min_width;
    let mut antipodal_index = 1_usize;
    for index in 0..vertex_count {
        let a = &points[index];
        let b = &points[(index + 1) % vertex_count];
        let edge_x = &b.0 - &a.0;
        let edge_y = &b.1 - &a.1;
        let edge_length_squared = &edge_x * &edge_x + &edge_y * &edge_y;
        if crate::scalar::eq(&edge_length_squared, &zero) {
            return false;
        }

        let signed_projection = |point: &(Scalar, Scalar)| {
            let projection = &edge_x * (&point.1 - &a.1) - &edge_y * (&point.0 - &a.0);
            if turn_direction > 0 {
                projection
            } else {
                -projection
            }
        };
        let mut span = signed_projection(&points[antipodal_index]);
        // Antipodal vertices advance monotonically around a convex polygon.
        // Strict comparison intentionally keeps the first vertex of a parallel
        // support edge; the next outer loop can still advance from that plateau.
        let mut advances = 0_usize;
        loop {
            let next_index = (antipodal_index + 1) % vertex_count;
            let next_span = signed_projection(&points[next_index]);
            if crate::scalar::le(&next_span, &span) {
                break;
            }
            antipodal_index = next_index;
            span = next_span;
            advances += 1;
            if advances >= vertex_count {
                return false;
            }
        }
        if crate::scalar::lt(&span, &zero) {
            return false;
        }
        if crate::scalar::lt(
            &(&span * &span),
            &(&min_width_squared * &edge_length_squared),
        ) {
            return false;
        }
    }
    true
}

/// Warn when one parsed layer contains duplicate polygon islands.
///
/// Intra-layer duplicates can come from repeated Gerber flashes, stale aperture
/// macro expansion, or CAM exports that wrote the same contour twice. The
/// geometry is usually harmless after boolean union, but it is still a release
/// readiness signal because duplicate primitives can confuse downstream CAM,
/// quoting, or review tools. The pairwise filter uses the same conservative
/// set-overlap idea as [`duplicate_layer_geometry_readiness`].
pub fn duplicate_layer_island_readiness(
    layer_name: &str,
    region: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    let multipolygon = region.to_multipolygon();
    let mut buckets = BTreeMap::<DuplicateIslandSignature, Vec<&Polygon<f64>>>::new();
    for polygon in &multipolygon.0 {
        if polygon_area_scalar(polygon).is_none_or(|area| crate::scalar::le(&area, min_area)) {
            continue;
        }
        let Some(signature) = duplicate_island_signature(polygon) else {
            continue;
        };
        buckets.entry(signature).or_default().push(polygon);
    }
    let comparable_polygons = buckets.values().map(Vec::len).sum::<usize>();
    log::trace!(
        "duplicate-layer island readiness: layer={layer_name} polygons={} comparable_polygons={} candidate_buckets={} min_area={min_area:#.9}",
        multipolygon.0.len(),
        comparable_polygons,
        buckets.values().filter(|bucket| bucket.len() > 1).count()
    );

    let mut locations = Vec::new();
    for bucket in buckets.values().filter(|bucket| bucket.len() > 1) {
        for left_index in 0..bucket.len() {
            let mut matched_left = false;
            for right_index in (left_index + 1)..bucket.len() {
                let duplicate = match polygons_are_duplicate(
                    bucket[left_index],
                    bucket[right_index],
                    &crate::scalar::scalar(BOARD_OUTLINE_GEOMETRY_TOLERANCE),
                    "layer-sanity",
                    vec![layer_name.to_string()],
                ) {
                    Ok(duplicate) => duplicate,
                    Err(uncertainty) => return vec![*uncertainty],
                };
                if duplicate {
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
/// file manifest. The overlap test remains intentionally conservative for CAM
/// handoff use.
pub fn duplicate_layer_geometry_readiness<T>(
    layers: &[(String, T)],
    min_area: &Scalar,
) -> Vec<Violation>
where
    T: std::borrow::Borrow<PcbRegion>,
{
    let prepared = layers
        .iter()
        .filter_map(|(name, region)| {
            let region = region.borrow();
            let multipolygon = region.to_multipolygon();
            let area = multipolygon_area_scalar(&multipolygon)?;
            let bounds = multipolygon_bounds_scalar(&multipolygon)?;
            (compare_reals_with_policy(&area, min_area, PredicatePolicy).value()
                == Some(std::cmp::Ordering::Greater))
            .then_some(DuplicateLayer {
                name,
                region,
                multipolygon,
                area,
                bounds,
            })
        })
        .collect::<Vec<_>>();
    log::trace!(
        "duplicate-layer geometry readiness: input_layers={} comparable_layers={} min_area={min_area:#.9}",
        layers.len(),
        prepared.len()
    );

    let mut violations = Vec::new();
    for left_index in 0..prepared.len() {
        for right_index in (left_index + 1)..prepared.len() {
            let left = &prepared[left_index];
            let right = &prepared[right_index];
            let geometry_tolerance = crate::scalar::scalar(BOARD_OUTLINE_GEOMETRY_TOLERANCE);
            if !areas_approximately_equal_scalar(&left.area, &right.area, &geometry_tolerance)
                || !bounds_approximately_equal_scalar(
                    &left.bounds,
                    &right.bounds,
                    &geometry_tolerance,
                )
            {
                continue;
            }

            let overlap = match intersection_for_check(
                left.region,
                right.region,
                "layer-sanity",
                vec![left.name.to_string(), right.name.to_string()],
            ) {
                Ok(overlap) => overlap,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let Some(overlap_area) = multipolygon_area_scalar(&overlap.to_multipolygon()) else {
                continue;
            };
            let Some(left_coverage) = (&overlap_area / &left.area).ok() else {
                continue;
            };
            let Some(right_coverage) = (&overlap_area / &right.area).ok() else {
                continue;
            };
            let overlap_ratio = crate::scalar::scalar(DUPLICATE_LAYER_OVERLAP_RATIO);
            if crate::scalar::lt(&left_coverage, &overlap_ratio)
                || crate::scalar::lt(&right_coverage, &overlap_ratio)
            {
                continue;
            }

            let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
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
                    if crate::scalar::le(&left_coverage, &right_coverage) {
                        left_coverage
                    } else {
                        right_coverage
                    }
                        * crate::scalar::scalar("100")
                )),
            ));
        }
    }

    violations
}

struct DuplicateLayer<'a> {
    name: &'a str,
    region: &'a PcbRegion,
    multipolygon: MultiPolygon<f64>,
    area: Scalar,
    bounds: [Scalar; 4],
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

fn bounds_approximately_equal_scalar(
    left: &[Scalar; 4],
    right: &[Scalar; 4],
    tolerance: &Scalar,
) -> bool {
    (0..4).all(|index| crate::scalar::le(&(&left[index] - &right[index]).abs(), tolerance))
}

fn multipolygon_bounds_scalar(multipolygon: &MultiPolygon<f64>) -> Option<[Scalar; 4]> {
    let mut polygon_bounds = multipolygon.0.iter().filter_map(polygon_bounds_scalar);
    let mut bounds = polygon_bounds.next()?;
    for candidate in polygon_bounds {
        if crate::scalar::lt(&candidate[0], &bounds[0]) {
            bounds[0] = candidate[0].clone();
        }
        if crate::scalar::lt(&candidate[1], &bounds[1]) {
            bounds[1] = candidate[1].clone();
        }
        if crate::scalar::gt(&candidate[2], &bounds[2]) {
            bounds[2] = candidate[2].clone();
        }
        if crate::scalar::gt(&candidate[3], &bounds[3]) {
            bounds[3] = candidate[3].clone();
        }
    }
    Some(bounds)
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
    copper_layers: &[(String, PcbRegion)],
    max_imbalance_ratio: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let mut measured = copper_layers
        .iter()
        .filter_map(|(name, region)| {
            let area = multipolygon_area_scalar(&region.to_multipolygon())?;
            crate::scalar::gt(&area, min_area).then_some((name.clone(), area))
        })
        .collect::<Vec<_>>();

    if measured.len() < 2 {
        return Vec::new();
    }

    measured.sort_by(|left, right| {
        crate::scalar::compare(&left.1, &right.1).expect("exact layer areas must be comparable")
    });
    let (smallest_layer, smallest_area) = &measured[0];
    let (largest_layer, largest_area) = &measured[measured.len() - 1];
    let Ok(ratio) = largest_area / smallest_area else {
        return Vec::new();
    };

    if crate::scalar::le(&ratio, max_imbalance_ratio) {
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
            "copper area imbalance ratio {ratio:#.3} exceeds maximum {max_imbalance_ratio:#.3}; smallest layer {smallest_layer} area {smallest_area:#.6}, largest layer {largest_layer} area {largest_area:#.6}"
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
/// The implementation uses rectangular windows so the result is deterministic,
/// inexpensive, and friendly to examples and tests.
pub fn local_copper_density_readiness(
    copper_layers: &[(String, PcbRegion)],
    window_size: &Scalar,
    max_density_ratio: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    if copper_layers.len() < 2 || crate::scalar::le(window_size, &Scalar::zero()) {
        return Vec::new();
    }

    let prepared_layers = prepare_density_layers(copper_layers, min_area);
    let Some(bounds) = combined_density_bounds(&prepared_layers) else {
        return Vec::new();
    };
    log::trace!(
        "local copper-density readiness: layers={} window_size={window_size:#.6} ratio={max_density_ratio:#.6}",
        prepared_layers.len()
    );

    let mut violations = Vec::new();
    let [min_x, min_y, max_x, max_y] = bounds;
    let mut y = min_y.clone();

    while crate::scalar::lt(&y, &max_y) {
        let mut x = min_x.clone();
        while crate::scalar::lt(&x, &max_x) {
            let remaining_width = &max_x - &x;
            let remaining_height = &max_y - &y;
            let width = if crate::scalar::le(&remaining_width, window_size) {
                remaining_width
            } else {
                window_size.clone()
            };
            let height = if crate::scalar::le(&remaining_height, window_size) {
                remaining_height
            } else {
                window_size.clone()
            };
            let window_area = &width * &height;
            if crate::scalar::gt(&window_area, min_area) {
                let two = crate::scalar::scalar("2");
                let center = [
                    (&x + (&width / &two).expect("two is nonzero")),
                    (&y + (&height / &two).expect("two is nonzero")),
                ];
                collect_local_density_window(
                    &prepared_layers,
                    center,
                    [width, height],
                    max_density_ratio,
                    min_area,
                    &mut violations,
                );
            }
            x = &x + window_size;
        }
        y = &y + window_size;
    }

    violations
}

struct DensityLayer {
    name: String,
    polygons: Vec<DensityPolygon>,
}

struct DensityPolygon {
    bounds: [Scalar; 4],
    area: Scalar,
}

fn prepare_density_layers(
    copper_layers: &[(String, PcbRegion)],
    min_area: &Scalar,
) -> Vec<DensityLayer> {
    copper_layers
        .iter()
        .filter_map(|(name, region)| {
            let polygons = region
                .to_multipolygon()
                .0
                .into_iter()
                .filter_map(|polygon| {
                    let area = polygon_area_scalar(&polygon)?;
                    let bounds = polygon_bounds_scalar(&polygon)?;
                    crate::scalar::gt(&area, min_area).then_some(DensityPolygon { bounds, area })
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
    center: [Scalar; 2],
    size: [Scalar; 2],
    max_density_ratio: &Scalar,
    min_area: &Scalar,
    violations: &mut Vec<Violation>,
) {
    let window_area = &size[0] * &size[1];
    if crate::scalar::le(&window_area, &Scalar::zero()) || crate::scalar::le(&window_area, min_area)
    {
        return;
    }
    let two = crate::scalar::scalar("2");
    let half_width = (&size[0] / &two).expect("two is nonzero");
    let half_height = (&size[1] / &two).expect("two is nonzero");
    let window_bounds = [
        &center[0] - &half_width,
        &center[1] - &half_height,
        &center[0] + &half_width,
        &center[1] + &half_height,
    ];
    let mut densities = copper_layers
        .iter()
        .map(|layer| {
            let copper_area =
                Scalar::sum_owned(layer.polygons.iter().filter_map(|polygon| {
                    approximate_window_polygon_area(&window_bounds, polygon)
                }));
            let mut density = (copper_area / &window_area)
                .expect("a policy-positive density window area is nonzero");
            if crate::scalar::lt(&density, &Scalar::zero()) {
                density = Scalar::zero();
            } else if crate::scalar::gt(&density, &crate::scalar::scalar("1")) {
                density = crate::scalar::scalar("1");
            }
            (&layer.name, density)
        })
        .collect::<Vec<_>>();

    densities.sort_by(|left, right| {
        crate::scalar::compare(&left.1, &right.1)
            .expect("exact copper densities must be comparable")
    });
    let Some((sparse_layer, sparse_density)) = densities.first() else {
        return;
    };
    let Some((dense_layer, dense_density)) = densities.last() else {
        return;
    };
    if crate::scalar::lt(dense_density, &crate::scalar::scalar("0.05")) {
        return;
    }

    let denominator = if crate::scalar::ge(sparse_density, min_area) {
        sparse_density
    } else {
        min_area
    };
    let Ok(ratio) = dense_density.clone() / denominator else {
        return;
    };
    let delta = dense_density - sparse_density;
    if crate::scalar::le(&ratio, max_density_ratio)
        || crate::scalar::lt(&delta, &crate::scalar::scalar("0.50"))
    {
        return;
    }

    let center_f64 = scalar_point_f64_compatibility(&center);
    let size_f64 = scalar_point_f64_compatibility(&size);
    let window_polygon = rect_polygon(center_f64, size_f64, 0.0);
    let window = polygons_to_profile(vec![window_polygon], Some(metadata("density window")));
    let shapes = multipolygon_to_shapes_scalar(&window.to_multipolygon(), min_area);
    violations.push(Violation::new(
        "local-copper-density-readiness",
        Severity::Warning,
        vec![(*sparse_layer).clone(), (*dense_layer).clone()],
        None,
        shapes,
        vec![center_f64],
        Some(format!(
            "local copper density imbalance ratio {ratio:#.3} exceeds maximum {max_density_ratio:#.3}; sparse layer {sparse_layer} density {sparse_density:#.3}, dense layer {dense_layer} density {dense_density:#.3}"
        )),
    ));
}

fn approximate_window_polygon_area(
    window: &[Scalar; 4],
    polygon: &DensityPolygon,
) -> Option<Scalar> {
    let overlap_min_x = if crate::scalar::ge(&window[0], &polygon.bounds[0]) {
        &window[0]
    } else {
        &polygon.bounds[0]
    };
    let overlap_min_y = if crate::scalar::ge(&window[1], &polygon.bounds[1]) {
        &window[1]
    } else {
        &polygon.bounds[1]
    };
    let overlap_max_x = if crate::scalar::le(&window[2], &polygon.bounds[2]) {
        &window[2]
    } else {
        &polygon.bounds[2]
    };
    let overlap_max_y = if crate::scalar::le(&window[3], &polygon.bounds[3]) {
        &window[3]
    } else {
        &polygon.bounds[3]
    };
    let overlap_width = overlap_max_x - overlap_min_x;
    let overlap_height = overlap_max_y - overlap_min_y;
    if crate::scalar::le(&overlap_width, &Scalar::zero())
        || crate::scalar::le(&overlap_height, &Scalar::zero())
    {
        return Some(Scalar::zero());
    }

    let bounds_area =
        (&polygon.bounds[2] - &polygon.bounds[0]) * (&polygon.bounds[3] - &polygon.bounds[1]);
    if crate::scalar::le(&bounds_area, &Scalar::zero()) {
        return Some(Scalar::zero());
    }

    // The local density check is a DFM heuristic, so it uses a conservative
    // raster-like area accumulator instead of exact CSG window clipping. This
    // mirrors the gridded density maps used for copper CMP/plating review while
    // avoiding a pathological "number of windows times number of layers" boolean
    // workload on large pours.
    (polygon.area.clone() * overlap_width * overlap_height / bounds_area).ok()
}

fn combined_density_bounds(copper_layers: &[DensityLayer]) -> Option<[Scalar; 4]> {
    copper_layers
        .iter()
        .flat_map(|layer| layer.polygons.iter().map(|polygon| polygon.bounds.clone()))
        .reduce(|left, right| {
            [
                if crate::scalar::le(&left[0], &right[0]) {
                    left[0].clone()
                } else {
                    right[0].clone()
                },
                if crate::scalar::le(&left[1], &right[1]) {
                    left[1].clone()
                } else {
                    right[1].clone()
                },
                if crate::scalar::ge(&left[2], &right[2]) {
                    left[2].clone()
                } else {
                    right[2].clone()
                },
                if crate::scalar::ge(&left[3], &right[3]) {
                    left[3].clone()
                } else {
                    right[3].clone()
                },
            ]
        })
}

fn scalar_point_f64_compatibility(point: &[Scalar; 2]) -> [f64; 2] {
    [
        point[0]
            .to_f64_lossy()
            .expect("density report x coordinate must be finite"),
        point[1]
            .to_f64_lossy()
            .expect("density report y coordinate must be finite"),
    ]
}

/// Run the `mechanical_layer_geometry` design-readiness check or report helper.
pub fn mechanical_layer_geometry(
    layer_name: &str,
    region: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    if !looks_like_mechanical_layer(layer_name) {
        return Vec::new();
    }

    let shapes = multipolygon_to_shapes_scalar(&region.to_multipolygon(), min_area);
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
    outline: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    let shapes = multipolygon_to_shapes_scalar(&outline.to_multipolygon(), min_area);
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
    outline: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    let shapes = multipolygon_to_shapes_scalar(&outline.to_multipolygon(), min_area);
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
    outline: &PcbRegion,
) -> Vec<Violation> {
    board_outline_self_intersection_readiness_with_grid(
        layer_name,
        outline,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Reject outline rings that self-intersect using retained source-grid facts.
///
/// The retained grid is provenance for the exact segment-classification gate.
/// It is not a report certificate by itself: the returned marker location is
/// still a compatibility coordinate for human review.
pub fn board_outline_self_intersection_readiness_with_grid(
    layer_name: &str,
    outline: &PcbRegion,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let intersections = collect_ring_self_intersections_with_grid(&outline.to_multipolygon(), grid);
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
pub fn board_outline_notch_readiness(layer_name: &str, outline: &PcbRegion) -> Vec<Violation> {
    board_outline_notch_readiness_with_grid(
        layer_name,
        outline,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Flag board-outline notches using retained source-grid facts.
///
/// The inside-corner/reflex decision is a topology predicate and is classified
/// through `hyperlimit::orient2` after lifting source coordinates. The final
/// notch angle remains a compatibility metric for reporting and thresholding at
/// the finite report boundary. Exact predicates decide topology;
/// approximate quantities stay named adapters.
pub fn board_outline_notch_readiness_with_grid(
    layer_name: &str,
    outline: &PcbRegion,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let mut locations = Vec::new();

    let multipolygon = outline.to_multipolygon();
    for polygon in &multipolygon.0 {
        collect_board_outline_notches_with_grid(polygon.exterior(), &mut locations, grid);
        for hole in polygon.interiors() {
            collect_board_outline_notches_with_grid(hole, &mut locations, grid);
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
pub fn board_outline_duplicate_readiness(layer_name: &str, outline: &PcbRegion) -> Vec<Violation> {
    board_outline_duplicate_readiness_with_grid(
        layer_name,
        outline,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Warn about duplicated outline contours using retained source-grid facts.
///
/// Exact lifted bounding-box predicates reject impossible contour pairs before
/// the existing CSG overlap-area report path, preserving report semantics.
pub fn board_outline_duplicate_readiness_with_grid(
    layer_name: &str,
    outline: &PcbRegion,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let mut locations = Vec::new();

    if let Err(uncertainty) = collect_board_outline_overlapping_exteriors(
        &outline.to_multipolygon(),
        &crate::scalar::scalar(BOARD_OUTLINE_DUPLICATE_OVERLAP_RATIO),
        &crate::scalar::scalar(BOARD_OUTLINE_GEOMETRY_TOLERANCE),
        false,
        grid,
        &mut locations,
        "board-outline-duplicate-readiness",
        vec![layer_name.to_string()],
    ) {
        return vec![*uncertainty];
    }

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
pub fn board_outline_nesting_readiness(layer_name: &str, outline: &PcbRegion) -> Vec<Violation> {
    board_outline_nesting_readiness_with_grid(
        layer_name,
        outline,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Warn about nested outline contours using retained source-grid facts.
///
/// The pair broad phase uses exact lifted bounding-box predicates before
/// falling through to compatibility CSG overlap, preserving source-grid
/// provenance at the decision boundary.
pub fn board_outline_nesting_readiness_with_grid(
    layer_name: &str,
    outline: &PcbRegion,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let mut locations = Vec::new();

    if let Err(uncertainty) = collect_board_outline_overlapping_exteriors(
        &outline.to_multipolygon(),
        &crate::scalar::scalar(BOARD_OUTLINE_NESTED_OVERLAP_RATIO),
        &crate::scalar::scalar(BOARD_OUTLINE_GEOMETRY_TOLERANCE),
        true,
        grid,
        &mut locations,
        "board-outline-nesting-readiness",
        vec![layer_name.to_string()],
    ) {
        return vec![*uncertainty];
    }

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
    left: &PcbRegion,
    right_name: &str,
    right: &PcbRegion,
    min_area: &Scalar,
) -> Vec<Violation> {
    let overlap = match intersection_for_check(
        left,
        right,
        spec.check,
        vec![left_name.to_string(), right_name.to_string()],
    ) {
        Ok(overlap) => overlap,
        Err(uncertainty) => return vec![*uncertainty],
    };
    let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
    if shapes.is_empty() {
        return Vec::new();
    }
    vec![Violation::new(
        spec.check,
        spec.severity,
        vec![left_name.to_string(), right_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some(spec.message.to_string()),
    )]
}

const BOARD_OUTLINE_NOTCH_ANGLE_DEGREES: &str = "300";
const BOARD_OUTLINE_GEOMETRY_TOLERANCE: &str = "0.000001";
const BOARD_OUTLINE_DUPLICATE_OVERLAP_RATIO: &str = "0.999999";
const BOARD_OUTLINE_NESTED_OVERLAP_RATIO: &str = "0.99999";

fn collect_ring_self_intersections(multipolygon: &MultiPolygon<f64>) -> Vec<[f64; 2]> {
    collect_ring_self_intersections_with_grid(multipolygon, SourceGridFacts::PRIMITIVE_FLOAT_EDGE)
}

fn collect_ring_self_intersections_with_grid(
    multipolygon: &MultiPolygon<f64>,
    grid: SourceGridFacts,
) -> Vec<[f64; 2]> {
    let mut locations = Vec::new();

    for polygon in &multipolygon.0 {
        collect_segment_self_intersections_with_grid(polygon.exterior(), &mut locations, grid);
        for hole in polygon.interiors() {
            collect_segment_self_intersections_with_grid(hole, &mut locations, grid);
        }
    }

    locations
}

fn collect_segment_self_intersections_with_grid(
    ring: &LineString<f64>,
    locations: &mut Vec<[f64; 2]>,
    grid: SourceGridFacts,
) {
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
            if !segment_bounding_boxes_overlap(
                coords[left],
                coords[(left + 1) % edge_count],
                coords[right],
                coords[(right + 1) % edge_count],
            ) {
                continue;
            }

            let intersection = ring_segment_intersection_with_grid(
                coords[left],
                coords[(left + 1) % edge_count],
                coords[right],
                coords[(right + 1) % edge_count],
                grid,
            );

            if let Some(location) = intersection {
                push_unique_location(locations, location);
            }
        }
    }
}

fn collect_board_outline_notches_with_grid(
    ring: &LineString<f64>,
    locations: &mut Vec<[f64; 2]>,
    grid: SourceGridFacts,
) {
    let coords = open_ring_coords(ring);
    if coords.len() < 3 {
        return;
    }

    let is_ccw = ring_is_ccw(ring);
    for index in 0..coords.len() {
        let previous = coords[(index + coords.len() - 1) % coords.len()];
        let current = coords[index];
        let next = coords[(index + 1) % coords.len()];

        let orientations = match is_ccw {
            Some(is_ccw) => [Some(is_ccw), None],
            None => [Some(false), Some(true)],
        };
        let interior_angle = orientations
            .into_iter()
            .flatten()
            .filter_map(|is_ccw| {
                board_outline_notch_interior_angle_with_grid(previous, current, next, is_ccw, grid)
            })
            .max_by(|left, right| {
                compare_reals_with_policy(left, right, PredicatePolicy)
                    .value()
                    .expect("exact notch angles must be comparable")
            });
        let Some(interior_angle) = interior_angle else {
            continue;
        };
        if compare_reals_with_policy(
            &interior_angle,
            &crate::scalar::scalar(BOARD_OUTLINE_NOTCH_ANGLE_DEGREES),
            PredicatePolicy,
        )
        .value()
            == Some(std::cmp::Ordering::Less)
        {
            continue;
        }

        push_unique_location(locations, [current.x, current.y]);
    }
}

#[allow(clippy::too_many_arguments)] // Keep exact-predicate context explicit at this audit boundary.
fn collect_board_outline_overlapping_exteriors(
    multipolygon: &MultiPolygon<f64>,
    containment_ratio: &Scalar,
    geometry_tolerance: &Scalar,
    detect_nesting: bool,
    grid: SourceGridFacts,
    locations: &mut Vec<[f64; 2]>,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<(), Box<Violation>> {
    let polygons = &multipolygon.0;
    if polygons.len() < 2 {
        return Ok(());
    }

    for outer_index in 0..polygons.len() {
        for inner_index in (outer_index + 1)..polygons.len() {
            let outer = &polygons[outer_index];
            let inner = &polygons[inner_index];
            if !polygon_bounding_rects_overlap_with_grid(outer, inner, grid) {
                continue;
            }

            if detect_nesting {
                if polygons_are_duplicate_with_grid(
                    outer,
                    inner,
                    geometry_tolerance,
                    grid,
                    requested_check,
                    layers.clone(),
                )? {
                    continue;
                }
                if polygon_contains_other_outer_with_grid(
                    outer,
                    inner,
                    containment_ratio,
                    geometry_tolerance,
                    grid,
                    requested_check,
                    layers.clone(),
                )? && let Some(point) = representative_point(inner)
                {
                    push_unique_location(locations, point);
                }

                if polygon_contains_other_outer_with_grid(
                    inner,
                    outer,
                    containment_ratio,
                    geometry_tolerance,
                    grid,
                    requested_check,
                    layers.clone(),
                )? && let Some(point) = representative_point(outer)
                {
                    push_unique_location(locations, point);
                }
            } else if polygons_are_duplicate_with_grid(
                outer,
                inner,
                geometry_tolerance,
                grid,
                requested_check,
                layers.clone(),
            )? && let Some(point) = representative_point(outer)
            {
                push_unique_location(locations, point);
            }
        }
    }
    Ok(())
}

fn polygons_are_duplicate(
    left: &Polygon<f64>,
    right: &Polygon<f64>,
    tolerance: &Scalar,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<bool, Box<Violation>> {
    polygons_are_duplicate_with_grid(
        left,
        right,
        tolerance,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
        requested_check,
        layers,
    )
}

fn polygons_are_duplicate_with_grid(
    left: &Polygon<f64>,
    right: &Polygon<f64>,
    tolerance: &Scalar,
    grid: SourceGridFacts,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<bool, Box<Violation>> {
    if !polygon_bounding_rects_overlap_with_grid(left, right, grid) {
        return Ok(false);
    }

    let Some(left_area) = polygon_area_scalar(left) else {
        return Ok(false);
    };
    let Some(right_area) = polygon_area_scalar(right) else {
        return Ok(false);
    };
    if crate::scalar::le(&left_area, &Scalar::zero())
        || crate::scalar::le(&right_area, &Scalar::zero())
    {
        return Ok(false);
    }

    if !areas_approximately_equal_scalar(&left_area, &right_area, tolerance) {
        return Ok(false);
    }

    let Some(overlap) = polygon_intersection_area_scalar(left, right, requested_check, layers)?
    else {
        return Ok(false);
    };
    let left_delta = (&left_area - &overlap).abs();
    let right_delta = (&right_area - &overlap).abs();
    Ok(
        crate::scalar::le(&left_delta, &tolerance_area_scalar(&left_area))
            && crate::scalar::le(&right_delta, &tolerance_area_scalar(&right_area)),
    )
}

fn polygon_contains_other_outer(
    outer: &Polygon<f64>,
    inner: &Polygon<f64>,
    ratio: &Scalar,
    tolerance: &Scalar,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<bool, Box<Violation>> {
    polygon_contains_other_outer_with_grid(
        outer,
        inner,
        ratio,
        tolerance,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
        requested_check,
        layers,
    )
}

fn polygon_contains_other_outer_with_grid(
    outer: &Polygon<f64>,
    inner: &Polygon<f64>,
    ratio: &Scalar,
    tolerance: &Scalar,
    grid: SourceGridFacts,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<bool, Box<Violation>> {
    if !polygon_bounding_rects_overlap_with_grid(outer, inner, grid) {
        return Ok(false);
    }

    let Some(outer_area) = polygon_area_scalar(outer) else {
        return Ok(false);
    };
    let Some(inner_area) = polygon_area_scalar(inner) else {
        return Ok(false);
    };
    if crate::scalar::le(&outer_area, &Scalar::zero())
        || crate::scalar::le(&inner_area, &Scalar::zero())
        || crate::scalar::le(&outer_area, &inner_area)
    {
        return Ok(false);
    }

    let Some(overlap) = polygon_intersection_area_scalar(outer, inner, requested_check, layers)?
    else {
        return Ok(false);
    };
    if crate::scalar::le(&overlap, &(&inner_area * crate::scalar::scalar("0.25"))) {
        return Ok(false);
    }

    let Some(coverage) = (&overlap / &inner_area).ok() else {
        return Ok(false);
    };
    let area_gap = &outer_area - &inner_area;
    Ok(crate::scalar::ge(&coverage, ratio)
        && crate::scalar::gt(&area_gap, &tolerance_area_scalar(&outer_area))
        && !areas_approximately_equal_scalar(&outer_area, &inner_area, tolerance))
}

fn polygon_intersection_area_scalar(
    left: &Polygon<f64>,
    right: &Polygon<f64>,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<Option<Scalar>, Box<Violation>> {
    let left_region = polygon_to_profile(left.clone(), None);
    let right_region = polygon_to_profile(right.clone(), None);
    let overlap = intersection_for_check(&left_region, &right_region, requested_check, layers)?;
    Ok(multipolygon_area_scalar(&overlap.to_multipolygon()))
}

fn representative_point(polygon: &Polygon<f64>) -> Option<[f64; 2]> {
    polygon.bounding_rect().map(|bounds| {
        [
            (bounds.min().x + bounds.max().x) / 2.0,
            (bounds.min().y + bounds.max().y) / 2.0,
        ]
    })
}

fn tolerance_area_scalar(area: &Scalar) -> Scalar {
    let scaled = area.abs() * crate::scalar::scalar("0.000000001");
    let floor = crate::scalar::scalar("0.000000000001");
    if crate::scalar::ge(&scaled, &floor) {
        scaled
    } else {
        floor
    }
}

fn areas_approximately_equal_scalar(
    left_area: &Scalar,
    right_area: &Scalar,
    tolerance: &Scalar,
) -> bool {
    let diff = (left_area - right_area).abs();
    let mut scale = left_area.abs();
    if crate::scalar::gt(&right_area.abs(), &scale) {
        scale = right_area.abs();
    }
    if crate::scalar::lt(&scale, &crate::scalar::scalar("1")) {
        scale = crate::scalar::scalar("1");
    }
    crate::scalar::le(&diff, &(tolerance * scale))
}

fn board_outline_notch_interior_angle_with_grid(
    previous: Coord<f64>,
    current: Coord<f64>,
    next: Coord<f64>,
    is_ccw: bool,
    grid: SourceGridFacts,
) -> Option<Scalar> {
    let provenance = RuleGeometryProvenance::new("board-outline-notch-readiness", grid);
    let previous_exact = lift_coord_with_provenance(previous, provenance)?;
    let current_exact = lift_coord_with_provenance(current, provenance)?;
    let next_exact = lift_coord_with_provenance(next, provenance)?;
    let first_x = &current_exact.x - &previous_exact.x;
    let first_y = &current_exact.y - &previous_exact.y;
    let second_x = &next_exact.x - &current_exact.x;
    let second_y = &next_exact.y - &current_exact.y;
    let first_length = (&first_x * &first_x + &first_y * &first_y).sqrt().ok()?;
    let second_length = (&second_x * &second_x + &second_y * &second_y)
        .sqrt()
        .ok()?;
    if crate::scalar::eq(&first_length, &Scalar::zero())
        || crate::scalar::eq(&second_length, &Scalar::zero())
    {
        return None;
    }
    let mut cosine =
        ((&first_x * &second_x + &first_y * &second_y) / (first_length * second_length)).ok()?;
    let negative_one = crate::scalar::scalar("-1");
    let one = crate::scalar::scalar("1");
    if crate::scalar::lt(&cosine, &negative_one) {
        cosine = negative_one;
    } else if crate::scalar::gt(&cosine, &one) {
        cosine = one;
    }
    let raw_angle = (cosine.acos().ok()? * crate::scalar::scalar("180") / Scalar::pi()).ok()?;
    let orientation = orient_coords_with_grid(previous, current, next, grid)?;
    let is_reflex = matches!(
        (is_ccw, orientation),
        (true, Sign::Negative) | (false, Sign::Positive)
    );
    if !is_reflex {
        return None;
    }

    Some(crate::scalar::scalar("360") - raw_angle)
}

fn orient_coords_with_grid(
    previous: Coord<f64>,
    current: Coord<f64>,
    next: Coord<f64>,
    grid: SourceGridFacts,
) -> Option<Sign> {
    let provenance = RuleGeometryProvenance::new("board-outline-notch-readiness", grid);
    let previous = lift_coord_with_provenance(previous, provenance)?;
    let current = lift_coord_with_provenance(current, provenance)?;
    let next = lift_coord_with_provenance(next, provenance)?;
    hyperlimit::orient2_with_policy(&previous, &current, &next, PredicatePolicy).value()
}

fn polygon_bounding_rects_overlap_with_grid(
    left: &Polygon<f64>,
    right: &Polygon<f64>,
    grid: SourceGridFacts,
) -> bool {
    let Some(left) = left.bounding_rect() else {
        return true;
    };
    let Some(right) = right.bounding_rect() else {
        return true;
    };

    // This broad phase rejects impossible CSG overlap, but its separating-axis
    // comparisons are exact lifted predicates with source-grid provenance.
    // Possible contacts are never rejected here.
    !exact_lt_with_grid(left.max().x, right.min().x, grid)
        && !exact_lt_with_grid(right.max().x, left.min().x, grid)
        && !exact_lt_with_grid(left.max().y, right.min().y, grid)
        && !exact_lt_with_grid(right.max().y, left.min().y, grid)
}

fn exact_lt_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> bool {
    exact_cmp_with_grid(left, right, grid)
        .is_some_and(|ordering| ordering == std::cmp::Ordering::Less)
}

fn exact_cmp_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> Option<std::cmp::Ordering> {
    let provenance = RuleGeometryProvenance::new("board-outline-overlap-readiness", grid);
    let left = provenance.lift_f64(left)?;
    let right = provenance.lift_f64(right)?;
    compare_reals_with_policy(&left, &right, PredicatePolicy).value()
}

fn ring_is_ccw(ring: &LineString<f64>) -> Option<bool> {
    let mut points = ring.0.iter();
    let first = points.next()?;
    let mut previous = first;
    let mut doubled = Scalar::zero();
    for point in points {
        let previous_x = Scalar::try_from(previous.x).ok()?;
        let previous_y = Scalar::try_from(previous.y).ok()?;
        let point_x = Scalar::try_from(point.x).ok()?;
        let point_y = Scalar::try_from(point.y).ok()?;
        doubled += previous_x * point_y - point_x * previous_y;
        previous = point;
    }
    if previous != first {
        doubled += Scalar::try_from(previous.x).ok()? * Scalar::try_from(first.y).ok()?
            - Scalar::try_from(first.x).ok()? * Scalar::try_from(previous.y).ok()?;
    }
    match crate::scalar::sign(&doubled)? {
        Sign::Positive => Some(true),
        Sign::Negative => Some(false),
        Sign::Zero => None,
    }
}

fn are_ring_edges_adjacent(left: usize, right: usize, edge_count: usize) -> bool {
    right == left + 1 || right + 1 == left || (left == 0 && right == edge_count - 1)
}

fn segment_bounding_boxes_overlap(
    start_a: Coord<f64>,
    end_a: Coord<f64>,
    start_b: Coord<f64>,
    end_b: Coord<f64>,
) -> bool {
    // Bounding volumes reject candidates before exact narrow predicates. This
    // f64 test only rejects pairs whose axis-aligned boxes are strictly
    // separated in the imported compatibility geometry; all possible contacts
    // still flow to the exact segment classifier below.
    let min_ax = start_a.x.min(end_a.x);
    let max_ax = start_a.x.max(end_a.x);
    let min_ay = start_a.y.min(end_a.y);
    let max_ay = start_a.y.max(end_a.y);
    let min_bx = start_b.x.min(end_b.x);
    let max_bx = start_b.x.max(end_b.x);
    let min_by = start_b.y.min(end_b.y);
    let max_by = start_b.y.max(end_b.y);

    max_ax >= min_bx && max_bx >= min_ax && max_ay >= min_by && max_by >= min_ay
}

fn ring_segment_intersection_with_grid(
    start_a: Coord<f64>,
    end_a: Coord<f64>,
    start_b: Coord<f64>,
    end_b: Coord<f64>,
    grid: SourceGridFacts,
) -> Option<[f64; 2]> {
    // Outline self-intersection is a topology readiness decision, not a visual
    // nicety. Classify the closed segments through `hyperlimit` before asking
    // Hypercurve also constructs the exact intersection point used for the
    // report marker; no second finite line-intersection algorithm is involved.
    let provenance = RuleGeometryProvenance::new("board-outline-self-intersection", grid);
    let a = lift_coord_with_provenance(start_a, provenance)?;
    let b = lift_coord_with_provenance(end_a, provenance)?;
    let c = lift_coord_with_provenance(start_b, provenance)?;
    let d = lift_coord_with_provenance(end_b, provenance)?;
    match hyperlimit::classify_segment_intersection_with_policy(&a, &b, &c, &d, PredicatePolicy)
        .value()
    {
        Some(SegmentIntersection::Disjoint | SegmentIntersection::EndpointTouch) => return None,
        Some(
            SegmentIntersection::Proper
            | SegmentIntersection::CollinearOverlap
            | SegmentIntersection::Identical,
        ) => {}
        None => return Some([(start_a.x + end_a.x) / 2.0, (start_a.y + end_a.y) / 2.0]),
    }

    let segment_a = LineSeg2::try_new(
        hypercurve::Point2::new(a.x, a.y),
        hypercurve::Point2::new(b.x, b.y),
    )
    .ok()?;
    let segment_b = LineSeg2::try_new(
        hypercurve::Point2::new(c.x, c.y),
        hypercurve::Point2::new(d.x, d.y),
    )
    .ok()?;
    match segment_a
        .intersect_line(&segment_b, &CurvePolicy::certified())
        .ok()?
    {
        LineLineIntersection::Point { point, kind, .. } => {
            if matches!(kind, hypercurve::IntersectionKind::Endpoint) {
                return None;
            }
            Some([point.x().to_f64_lossy()?, point.y().to_f64_lossy()?])
        }
        LineLineIntersection::Overlap { segment, .. } => Some([
            crate::scalar::half(&(segment.start().x() + segment.end().x())).to_f64_lossy()?,
            crate::scalar::half(&(segment.start().y() + segment.end().y())).to_f64_lossy()?,
        ]),
        LineLineIntersection::None => None,
        LineLineIntersection::Uncertain { .. } => {
            Some([(start_a.x + end_a.x) / 2.0, (start_a.y + end_a.y) / 2.0])
        }
    }
}

fn lift_coord_with_provenance(
    coord: Coord<f64>,
    provenance: RuleGeometryProvenance,
) -> Option<Point2> {
    Some(Point2::new(
        provenance.lift_f64(coord.x)?,
        provenance.lift_f64(coord.y)?,
    ))
}

fn push_unique_location(points: &mut Vec<[f64; 2]>, point: [f64; 2]) {
    if !points
        .iter()
        .any(|current| locations_are_equal(current, &point))
    {
        points.push(point);
    }
}

fn locations_are_equal(left: &[f64; 2], right: &[f64; 2]) -> bool {
    let provenance = RuleGeometryProvenance::new(
        "layer-report-location-deduplication",
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    );
    let Some(left) = lift_coord_with_provenance(
        Coord {
            x: left[0],
            y: left[1],
        },
        provenance,
    ) else {
        return false;
    };
    let Some(right) = lift_coord_with_provenance(
        Coord {
            x: right[0],
            y: right[1],
        },
        provenance,
    ) else {
        return false;
    };
    hyperlimit::point2_equal_with_policy(&left, &right, PredicatePolicy)
        .value()
        .unwrap_or(false)
}

/// Project an exact rule distance only for conservative finite spatial-index
/// candidate generation. The following representable float avoids dropping a
/// true boundary candidate when the exact value rounds downward.
fn scalar_broad_phase_radius(value: &Scalar) -> f64 {
    let projected = value
        .to_f64_lossy()
        .expect("layer broad-phase radius must fit the finite compatibility index");
    if projected > 0.0 {
        projected.next_up()
    } else {
        0.0
    }
}

#[derive(Clone)]
enum IndexedDifferenceMode {
    CoverAsIs,
    CoverOffset(Scalar),
}

#[derive(Clone)]
enum IndexedCoverMode {
    AsIs,
    Offset(Scalar),
    OffsetRing(Scalar),
}

struct ExactRegionComponent {
    region: PcbRegion,
    conservative_finite_bounds: [f64; 4],
}

fn exact_region_components(region: &PcbRegion) -> Option<Vec<ExactRegionComponent>> {
    let regions = region.exact_component_regions()?;
    regions
        .iter()
        .cloned()
        .map(|component_region| {
            let component = PcbRegion::new_shared(component_region, region.metadata().clone());
            Some(ExactRegionComponent {
                conservative_finite_bounds: conservative_exact_region_bounds(&component)?,
                region: component,
            })
        })
        .collect()
}

fn exact_region_components_or_whole(region: &PcbRegion) -> Option<Vec<ExactRegionComponent>> {
    if matches!(
        region.loop_role_counts(&hypercurve::CurvePolicy::certified()),
        Ok(hypercurve::Classification::Decided((1, 0)))
    ) {
        return Some(vec![ExactRegionComponent {
            region: region.clone(),
            conservative_finite_bounds: conservative_exact_region_bounds(region)?,
        }]);
    }
    if let Some(components) = exact_region_components(region) {
        return Some(components);
    }
    let bounds = region.to_multipolygon().bounding_rect()?;
    let points = [
        [bounds.min().x, bounds.min().y],
        [bounds.max().x, bounds.max().y],
    ];
    Some(vec![ExactRegionComponent {
        region: region.clone(),
        conservative_finite_bounds: conservative_projection_bounds(&points, 1.0e-3)?,
    }])
}

/// Conservatively project an exact retained region AABB for broad-phase use.
///
/// The exact HyperCurve bounds, rather than the compatibility polygon
/// projection, determine component identity. Expanding the lossy scalar
/// projection by one representable float keeps rejection conservative without
/// coupling exact decomposition to tessellation output.
fn conservative_exact_region_bounds(region: &PcbRegion) -> Option<[f64; 4]> {
    let bounds = region.bounding_box();
    let min_x = bounds.mins.x.to_f64_lossy()?;
    let min_y = bounds.mins.y.to_f64_lossy()?;
    let max_x = bounds.maxs.x.to_f64_lossy()?;
    let max_y = bounds.maxs.y.to_f64_lossy()?;
    if !min_x.is_finite() || !min_y.is_finite() || !max_x.is_finite() || !max_y.is_finite() {
        return None;
    }
    Some([
        min_x.next_down(),
        min_y.next_down(),
        max_x.next_up(),
        max_y.next_up(),
    ])
}

/// Conservative finite bounds for exact-component candidate rejection.
///
/// HyperCurve's projection tolerance bounds the emitted chord's deviation from
/// its retained curve. Expanding every side by that budget and one
/// representable float means a strict separation can only reject an exact
/// curve pair that cannot meet. These bounds never certify a topology result.
fn conservative_projection_bounds(points: &[[f64; 2]], error: f64) -> Option<[f64; 4]> {
    let first = *points.first()?;
    if !first[0].is_finite() || !first[1].is_finite() {
        return None;
    }
    let mut bounds = [first[0], first[1], first[0], first[1]];
    for point in &points[1..] {
        if !point[0].is_finite() || !point[1].is_finite() {
            return None;
        }
        bounds[0] = bounds[0].min(point[0]);
        bounds[1] = bounds[1].min(point[1]);
        bounds[2] = bounds[2].max(point[0]);
        bounds[3] = bounds[3].max(point[1]);
    }
    Some([
        (bounds[0] - error).next_down(),
        (bounds[1] - error).next_down(),
        (bounds[2] + error).next_up(),
        (bounds[3] + error).next_up(),
    ])
}

fn conservative_finite_bounds_are_disjoint(left: [f64; 4], right: [f64; 4]) -> bool {
    left[2] < right[0] || right[2] < left[0] || left[3] < right[1] || right[3] < left[1]
}

/// Compute a difference by exact retained-region component.
///
/// Certified projection envelopes only reject provably disjoint component
/// pairs. All candidates flow through the regularized HyperCurve Boolean, and
/// all output loops remain authoritative retained curves.
fn exact_componentwise_difference(
    requested_check: &str,
    subject_name: &str,
    subject: &PcbRegion,
    cover_name: &str,
    cover: &PcbRegion,
    cover_offset: Option<&Scalar>,
) -> Option<Result<PcbRegion, Box<Violation>>> {
    let subjects = exact_region_components(subject)?;
    let covers = exact_region_components(cover)?;
    exact_componentwise_difference_from_components(
        requested_check,
        subject_name,
        subject.metadata(),
        subjects,
        cover_name,
        covers,
        cover_offset,
    )
}

fn exact_componentwise_difference_from_components(
    requested_check: &str,
    subject_name: &str,
    subject_metadata: &Option<LayerMetadata>,
    subjects: Vec<ExactRegionComponent>,
    cover_name: &str,
    mut covers: Vec<ExactRegionComponent>,
    cover_offset: Option<&Scalar>,
) -> Option<Result<PcbRegion, Box<Violation>>> {
    // Small local land patterns usually decide an aperture before a plane or
    // zone spanning most of the board. Preserve exact subtraction semantics
    // while deferring expensive large-component offsets until they are
    // genuinely needed.
    covers.sort_by(|left, right| {
        let left_width = left.conservative_finite_bounds[2] - left.conservative_finite_bounds[0];
        let left_height = left.conservative_finite_bounds[3] - left.conservative_finite_bounds[1];
        let right_width = right.conservative_finite_bounds[2] - right.conservative_finite_bounds[0];
        let right_height =
            right.conservative_finite_bounds[3] - right.conservative_finite_bounds[1];
        (left_width * left_height).total_cmp(&(right_width * right_height))
    });
    let offset_covers = cover_offset
        .map(|_| {
            (0..covers.len())
                .map(|_| std::cell::OnceCell::<PcbRegion>::new())
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();
    if let Some(distance) = cover_offset {
        let radius = distance
            .to_f64_lossy()
            .filter(|radius| radius.is_finite())
            .map(f64::abs)?;
        for component in &mut covers {
            component.conservative_finite_bounds = [
                (component.conservative_finite_bounds[0] - radius).next_down(),
                (component.conservative_finite_bounds[1] - radius).next_down(),
                (component.conservative_finite_bounds[2] + radius).next_up(),
                (component.conservative_finite_bounds[3] + radius).next_up(),
            ];
        }
    }
    let mut combined_remainder: Option<PcbRegion> = None;

    for component in subjects {
        let mut remainder = Some(component.region);
        for (cover_index, cover) in covers.iter().enumerate() {
            if conservative_finite_bounds_are_disjoint(
                component.conservative_finite_bounds,
                cover.conservative_finite_bounds,
            ) {
                continue;
            }
            let cover_region = if let Some(distance) = cover_offset {
                let offset = &offset_covers[cover_index];
                if offset.get().is_none() {
                    let region = match offset_for_check(
                        &cover.region,
                        distance.clone(),
                        requested_check,
                        vec![subject_name.to_string(), cover_name.to_string()],
                    ) {
                        Ok(region) => region,
                        Err(uncertainty) => return Some(Err(uncertainty)),
                    };
                    let _ = offset.set(region);
                }
                offset
                    .get()
                    .expect("candidate cover offset was initialized")
            } else {
                &cover.region
            };
            let current = remainder
                .as_ref()
                .expect("nonempty exact component remainder is available");
            let next = match difference_for_check(
                current,
                cover_region,
                requested_check,
                vec![subject_name.to_string(), cover_name.to_string()],
            ) {
                Ok(remainder) => remainder,
                Err(uncertainty) => return Some(Err(uncertainty)),
            };
            if next.is_empty() {
                remainder = None;
                break;
            }
            remainder = Some(next);
        }
        if let Some(remainder) = remainder {
            combined_remainder = Some(match combined_remainder {
                Some(combined) => match union_for_check(
                    &combined,
                    &remainder,
                    requested_check,
                    vec![subject_name.to_string(), cover_name.to_string()],
                ) {
                    Ok(union) => union,
                    Err(uncertainty) => return Some(Err(uncertainty)),
                },
                None => remainder,
            });
        }
    }

    Some(Ok(combined_remainder.unwrap_or_else(|| {
        PcbRegion::new(CurveRegion2::empty(), subject_metadata.clone())
    })))
}

fn indexed_difference(
    requested_check: &str,
    subject_name: &str,
    subject: &PcbRegion,
    cover_name: &str,
    cover: &PcbRegion,
    search_radius: f64,
    mode: IndexedDifferenceMode,
) -> Result<PcbRegion, Box<Violation>> {
    let subject_polygons = subject.to_multipolygon().0;
    let subject_count = subject_polygons.len();
    let cover_polygons = cover.to_multipolygon().0;
    let cover_index = LayerPolygonSpatialIndex::new(&cover_polygons, search_radius);
    let cover_cache = (0..cover_polygons.len())
        .map(|_| std::cell::OnceCell::<PcbRegion>::new())
        .collect::<Vec<_>>();
    let distributes_over_candidates = match &mode {
        IndexedDifferenceMode::CoverAsIs => true,
        IndexedDifferenceMode::CoverOffset(distance) => {
            crate::scalar::ge(distance, &Scalar::zero())
        }
    };
    let mut remainder_polygons = Vec::new();
    let mut candidate_polygons = 0usize;

    for subject_polygon in subject_polygons {
        let candidates = cover_index.candidates_near_polygon(&subject_polygon, search_radius);
        candidate_polygons += candidates.len();
        let subject_island =
            polygon_to_profile(subject_polygon.clone(), Some(metadata(subject_name)));

        if candidates.is_empty() {
            remainder_polygons.push(subject_polygon);
            continue;
        }

        let mut candidates = candidates;
        candidates.sort_by(|left, right| {
            let area = |index: usize| {
                cover_polygons[index]
                    .bounding_rect()
                    .map_or(f64::INFINITY, |bounds| {
                        bounds.width().abs() * bounds.height().abs()
                    })
            };
            area(*left).total_cmp(&area(*right))
        });
        let remainder = if distributes_over_candidates {
            let mut remainder = subject_island;
            for candidate in candidates {
                let cached = &cover_cache[candidate];
                if cached.get().is_none() {
                    let mut candidate_region = polygon_to_profile(
                        cover_polygons[candidate].clone(),
                        Some(metadata(cover_name)),
                    );
                    if let IndexedDifferenceMode::CoverOffset(distance) = &mode {
                        candidate_region = offset_for_check(
                            &candidate_region,
                            distance.clone(),
                            requested_check,
                            vec![subject_name.to_string(), cover_name.to_string()],
                        )?;
                    }
                    let _ = cached.set(candidate_region);
                }
                remainder = difference_for_check(
                    &remainder,
                    cached
                        .get()
                        .expect("indexed cover candidate was initialized"),
                    requested_check,
                    vec![subject_name.to_string(), cover_name.to_string()],
                )?;
                if remainder.is_empty() {
                    break;
                }
            }
            remainder
        } else {
            let mut candidates = candidates.into_iter();
            let Some(first) = candidates.next() else {
                remainder_polygons.push(subject_polygon);
                continue;
            };
            let mut combined =
                polygon_to_profile(cover_polygons[first].clone(), Some(metadata(cover_name)));
            for candidate in candidates {
                let candidate = polygon_to_profile(
                    cover_polygons[candidate].clone(),
                    Some(metadata(cover_name)),
                );
                combined = union_for_check(
                    &combined,
                    &candidate,
                    requested_check,
                    vec![subject_name.to_string(), cover_name.to_string()],
                )?;
            }
            let IndexedDifferenceMode::CoverOffset(distance) = &mode else {
                unreachable!("only inward offsets require combined candidate geometry");
            };
            let combined = offset_for_check(
                &combined,
                distance.clone(),
                requested_check,
                vec![subject_name.to_string(), cover_name.to_string()],
            )?;
            difference_for_check(
                &subject_island,
                &combined,
                requested_check,
                vec![subject_name.to_string(), cover_name.to_string()],
            )?
        };
        remainder_polygons.extend(remainder.to_multipolygon().0);
    }

    log::trace!(
        "indexed layer difference: subject={subject_name} subject_islands={} cover={cover_name} cover_islands={} cover_buckets={} candidate_polygons={} search_radius={search_radius:.6}",
        subject_count,
        cover_polygons.len(),
        cover_index.bucket_count(),
        candidate_polygons
    );

    Ok(polygons_to_profile(
        remainder_polygons,
        Some(metadata(subject_name)),
    ))
}

fn indexed_intersection(
    requested_check: &str,
    subject_name: &str,
    subject: &PcbRegion,
    cover_name: &str,
    cover: &PcbRegion,
    search_radius: f64,
) -> Result<PcbRegion, Box<Violation>> {
    indexed_intersection_with_mode(
        requested_check,
        subject_name,
        subject,
        cover_name,
        cover,
        search_radius,
        IndexedCoverMode::AsIs,
    )
}

fn indexed_intersection_with_mode(
    requested_check: &str,
    subject_name: &str,
    subject: &PcbRegion,
    cover_name: &str,
    cover: &PcbRegion,
    search_radius: f64,
    mode: IndexedCoverMode,
) -> Result<PcbRegion, Box<Violation>> {
    let subject_polygons = subject.to_multipolygon().0;
    let subject_count = subject_polygons.len();
    let cover_polygons = cover.to_multipolygon().0;
    let cover_index = LayerPolygonSpatialIndex::new(&cover_polygons, search_radius);
    let mut overlap_polygons = Vec::new();
    let mut candidate_polygons = 0usize;

    for subject_polygon in subject_polygons {
        let candidates = cover_index.candidates_near_polygon(&subject_polygon, search_radius);
        candidate_polygons += candidates.len();
        if candidates.is_empty() {
            continue;
        }

        let subject_island = polygon_to_profile(subject_polygon, Some(metadata(subject_name)));
        let cover_candidates = candidates
            .into_iter()
            .map(|index| cover_polygons[index].clone())
            .collect::<Vec<_>>();
        let cover_region = polygons_to_profile(cover_candidates, Some(metadata(cover_name)));
        let cover_region = match mode {
            IndexedCoverMode::AsIs => cover_region,
            IndexedCoverMode::Offset(ref distance) => offset_for_check(
                &cover_region,
                distance.clone(),
                requested_check,
                vec![subject_name.to_string(), cover_name.to_string()],
            )?,
            IndexedCoverMode::OffsetRing(ref distance) => {
                let expanded = offset_for_check(
                    &cover_region,
                    distance.clone(),
                    requested_check,
                    vec![subject_name.to_string(), cover_name.to_string()],
                )?;
                difference_for_check(
                    &expanded,
                    &cover_region,
                    requested_check,
                    vec![subject_name.to_string(), cover_name.to_string()],
                )?
            }
        };
        let overlap = intersection_for_check(
            &subject_island,
            &cover_region,
            requested_check,
            vec![subject_name.to_string(), cover_name.to_string()],
        )?;
        overlap_polygons.extend(overlap.to_multipolygon().0);
    }

    log::trace!(
        "indexed layer intersection: subject={subject_name} subject_islands={} cover={cover_name} cover_islands={} cover_buckets={} candidate_polygons={} search_radius={search_radius:.6}",
        subject_count,
        cover_polygons.len(),
        cover_index.bucket_count(),
        candidate_polygons
    );

    Ok(polygons_to_profile(
        overlap_polygons,
        Some(metadata(subject_name)),
    ))
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
    region: PcbRegion,
    min_area: &Scalar,
    message: String,
) -> Vec<Violation> {
    let shapes = multipolygon_to_shapes_scalar(&region.to_multipolygon(), min_area);

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

fn shapes_violation_scalar(
    check: &str,
    severity: Severity,
    layers: Vec<String>,
    region: PcbRegion,
    min_area: &Scalar,
    message: String,
) -> Vec<Violation> {
    let shapes = multipolygon_to_shapes_scalar(&region.to_multipolygon(), min_area);

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
    max_angle_degrees: &Scalar,
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
        let Some(angle) = angle_degrees_scalar(previous, current, next) else {
            continue;
        };

        if crate::scalar::gt(&angle, &Scalar::zero())
            && crate::scalar::lt(&angle, max_angle_degrees)
        {
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

fn angle_degrees_scalar(
    previous: Coord<f64>,
    current: Coord<f64>,
    next: Coord<f64>,
) -> Option<Scalar> {
    let current_x = Scalar::try_from(current.x).ok()?;
    let current_y = Scalar::try_from(current.y).ok()?;
    let ax = Scalar::try_from(previous.x).ok()? - &current_x;
    let ay = Scalar::try_from(previous.y).ok()? - &current_y;
    let bx = Scalar::try_from(next.x).ok()? - current_x;
    let by = Scalar::try_from(next.y).ok()? - current_y;
    let a_len = (&ax * &ax + &ay * &ay).sqrt().ok()?;
    let b_len = (&bx * &bx + &by * &by).sqrt().ok()?;
    if crate::scalar::eq(&a_len, &Scalar::zero()) || crate::scalar::eq(&b_len, &Scalar::zero()) {
        return Some(Scalar::zero());
    }
    let denominator = a_len * b_len;
    let mut cosine = ((&ax * &bx + &ay * &by) / denominator).ok()?;
    let negative_one = crate::scalar::scalar("-1");
    let one = crate::scalar::scalar("1");
    if crate::scalar::lt(&cosine, &negative_one) {
        cosine = negative_one;
    } else if crate::scalar::gt(&cosine, &one) {
        cosine = one;
    }
    let radians = cosine.acos().ok()?;
    (radians * crate::scalar::scalar("180") / Scalar::pi()).ok()
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
pub(crate) mod tests {
    use std::path::Path;
    use std::process::Command;
    use std::time::{Duration, Instant};

    use crate::geometry::{Coord, LineString, Polygon};

    use super::{
        acid_trap_candidates, board_edge_clearance, board_outline_cutout_clearance,
        board_outline_duplicate_readiness, board_outline_duplicate_readiness_with_grid,
        board_outline_fragments, board_outline_nesting_readiness,
        board_outline_nesting_readiness_with_grid, board_outline_notch_readiness,
        board_outline_notch_readiness_with_grid, board_outline_sanity,
        board_outline_self_intersection_readiness,
        board_outline_self_intersection_readiness_with_grid, convex_polygon_width_at_least,
        copper_balance, copper_overlap, copper_overlap_with_ipc356,
        duplicate_layer_geometry_readiness, duplicate_layer_island_readiness, exposed_copper,
        layer_sanity, local_copper_density_readiness, locations_are_equal, mask_island_keepout,
        mechanical_layer_geometry, min_copper_neck_width, minimum_mask_opening,
        minimum_paste_aperture, paste_aperture_coverage, paste_aperture_ratio,
        paste_aperture_spacing, paste_mask_alignment, paste_overhang, paste_overhang_from_features,
        silkscreen_board_edge_clearance, silkscreen_clearance, silkscreen_min_width,
        silkscreen_overlap, silkscreen_text_height_readiness, skinny_layer_feature_readiness,
        solder_mask_annular_ring_readiness, solder_mask_board_edge_clearance,
        solder_mask_expansion, solder_mask_expansion_from_features, solder_mask_opening_coverage,
        solder_mask_opening_ratio_readiness, solder_mask_opening_spacing,
        solder_mask_overlap_clearance, solder_mask_sliver, tiny_layer_feature_readiness,
    };
    use crate::LayerMetadata;
    use crate::geometry::{
        SourceGridFacts, SourceUnit, empty_profile, line_polygon, polygons_to_profile, rect_polygon,
    };
    use crate::ipc356::Ipc356Point;
    use crate::kicad::load_kicad_pcb;

    #[test]
    fn report_location_deduplication_does_not_merge_nearby_exact_points() {
        assert!(locations_are_equal(&[1.0, 2.0], &[1.0, 2.0]));
        assert!(!locations_are_equal(&[1.0, 2.0], &[1.0 + 5.0e-10, 2.0]));
    }

    #[test]
    fn mask_island_keepout_reports_expanded_island_collision() {
        let layer = region(
            "mask",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.1, 0.0, 2.1, 1.0)],
        );

        let violations = mask_island_keepout(
            "mask",
            &layer,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            violations
                .iter()
                .all(|violation| violation.total_area > 0.0)
        );
    }

    #[test]
    fn mask_island_keepout_allows_distant_islands() {
        let layer = region(
            "mask",
            vec![square(0.0, 0.0, 1.0, 1.0), square(5.0, 0.0, 6.0, 1.0)],
        );

        let violations = mask_island_keepout(
            "mask",
            &layer,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn mask_island_keepout_culls_sparse_island_fields() {
        let mut islands = (0..2_000)
            .map(|index| {
                let x = 100.0 + index as f64 * 3.0;
                square(x, 10.0, x + 0.5, 10.5)
            })
            .collect::<Vec<_>>();
        islands.push(square(0.0, 0.0, 1.0, 1.0));
        islands.push(square(1.1, 0.0, 2.1, 1.0));
        let layer = region("mask", islands);

        let start = Instant::now();
        let violations = mask_island_keepout(
            "mask",
            &layer,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "mask-island keepout should index sparse island fields"
        );
    }

    #[test]
    fn copper_overlap_reports_intersection_coordinates() {
        let top = region("top", vec![square(0.0, 0.0, 2.0, 2.0)]);
        let bottom = region("bottom", vec![square(1.0, 1.0, 3.0, 3.0)]);

        let violations = copper_overlap(
            "top",
            &top,
            "bottom",
            &bottom,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].polygons.len(), 1);
        assert!((violations[0].polygons[0].area - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn copper_overlap_with_ipc356_marks_same_net_evidence_as_review_warning() {
        let top = region("top", vec![square(0.0, 0.0, 2.0, 2.0)]);
        let bottom = region("bottom", vec![square(1.0, 1.0, 3.0, 3.0)]);
        let points = vec![ipc_point("GND", [1.5, 1.5])];

        let violations = copper_overlap_with_ipc356(
            "top",
            &top,
            "bottom",
            &bottom,
            &points,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].severity, crate::report::Severity::Warning);
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("same-net evidence for GND"))
        );
    }

    #[test]
    fn copper_overlap_with_ipc356_reports_mixed_net_evidence_as_error() {
        let top = region("top", vec![square(0.0, 0.0, 2.0, 2.0)]);
        let bottom = region("bottom", vec![square(1.0, 1.0, 3.0, 3.0)]);
        let points = vec![ipc_point("GND", [1.5, 1.5]), ipc_point("VBUS", [1.6, 1.6])];

        let violations = copper_overlap_with_ipc356(
            "top",
            &top,
            "bottom",
            &bottom,
            &points,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].severity, crate::report::Severity::Error);
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("mixed IPC-D-356 net evidence"))
        );
    }

    #[test]
    fn copper_balance_reports_large_area_imbalance() {
        let layers = vec![
            (
                "F.Cu".to_string(),
                region("F.Cu", vec![square(0.0, 0.0, 1.0, 1.0)]),
            ),
            (
                "B.Cu".to_string(),
                region("B.Cu", vec![square(0.0, 0.0, 4.0, 4.0)]),
            ),
        ];

        let violations = copper_balance(
            &layers,
            &crate::scalar::scalar("3.0"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "copper-balance-readiness");
    }

    #[test]
    fn copper_balance_allows_similar_or_single_sided_inputs() {
        let balanced = vec![
            (
                "F.Cu".to_string(),
                region("F.Cu", vec![square(0.0, 0.0, 2.0, 2.0)]),
            ),
            (
                "B.Cu".to_string(),
                region("B.Cu", vec![square(0.0, 0.0, 2.5, 2.0)]),
            ),
        ];
        let single = vec![(
            "F.Cu".to_string(),
            region("F.Cu", vec![square(0.0, 0.0, 2.0, 2.0)]),
        )];

        assert!(
            copper_balance(
                &balanced,
                &crate::scalar::scalar("3.0"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
        assert!(
            copper_balance(
                &single,
                &crate::scalar::scalar("3.0"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn local_copper_density_reports_dense_island_over_sparse_layer() {
        let layers = vec![
            (
                "F.Cu".to_string(),
                polygons_to_profile(
                    vec![rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
                    Some(LayerMetadata {
                        name: "F.Cu".to_string(),
                    }),
                ),
            ),
            (
                "B.Cu".to_string(),
                polygons_to_profile(
                    vec![rect_polygon([5.0, 5.0], [1.0, 1.0], 0.0)],
                    Some(LayerMetadata {
                        name: "B.Cu".to_string(),
                    }),
                ),
            ),
        ];

        let violations = local_copper_density_readiness(
            &layers,
            &crate::scalar::scalar("10.0"),
            &crate::scalar::scalar("3.0"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
                polygons_to_profile(
                    vec![rect_polygon([5.0, 5.0], [7.0, 7.0], 0.0)],
                    Some(LayerMetadata {
                        name: "F.Cu".to_string(),
                    }),
                ),
            ),
            (
                "B.Cu".to_string(),
                polygons_to_profile(
                    vec![rect_polygon([5.0, 5.0], [6.0, 6.0], 0.0)],
                    Some(LayerMetadata {
                        name: "B.Cu".to_string(),
                    }),
                ),
            ),
        ];

        assert!(
            local_copper_density_readiness(
                &layers,
                &crate::scalar::scalar("10.0"),
                &crate::scalar::scalar("3.0"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn board_edge_clearance_reports_copper_outside_eroded_outline() {
        let board = region("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let copper = region("top", vec![square(0.1, 0.1, 1.0, 1.0)]);

        let violations = board_edge_clearance(
            "top",
            &copper,
            "edge",
            &board,
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn silkscreen_board_edge_clearance_reports_legend_near_edge() {
        let board = region("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let silk = region("silk", vec![square(0.1, 0.1, 1.0, 1.0)]);

        let violations = silkscreen_board_edge_clearance(
            "silk",
            &silk,
            "edge",
            &board,
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "silkscreen-to-board-edge-clearance");
    }

    #[test]
    fn silkscreen_board_edge_clearance_allows_inset_legend() {
        let board = region("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let silk = region("silk", vec![square(1.0, 1.0, 2.0, 2.0)]);

        assert!(
            silkscreen_board_edge_clearance(
                "silk",
                &silk,
                "edge",
                &board,
                &crate::scalar::scalar("0.25"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn solder_mask_board_edge_clearance_reports_opening_near_edge() {
        let board = region("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let mask = region("mask", vec![square(0.1, 0.1, 1.0, 1.0)]);

        let violations = solder_mask_board_edge_clearance(
            "mask",
            &mask,
            "edge",
            &board,
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-to-board-edge-clearance");
    }

    #[test]
    fn solder_mask_board_edge_clearance_allows_inset_opening() {
        let board = region("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let mask = region("mask", vec![square(1.0, 1.0, 2.0, 2.0)]);

        assert!(
            solder_mask_board_edge_clearance(
                "mask",
                &mask,
                "edge",
                &board,
                &crate::scalar::scalar("0.25"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn paste_overhang_reports_paste_outside_copper() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let paste = region("paste", vec![square(-0.1, 0.0, 1.0, 1.0)]);

        let violations = paste_overhang(
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("0.0"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn paste_overhang_regularizes_overlapping_exact_cover_candidates() {
        let copper = region(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(0.5, 0.0, 1.5, 1.0)],
        );
        let paste = region("paste", vec![square(0.25, 0.2, 1.25, 0.8)]);

        let violations = paste_overhang(
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("0.0"),
            &crate::scalar::scalar("1.0e-9"),
        );
        assert!(violations.is_empty(), "{violations:#?}");
    }

    #[test]
    fn paste_overhang_culls_sparse_copper_fields() {
        let copper = region(
            "top",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(0.0, 0.0, 0.8, 1.0)])
                .collect(),
        );
        let paste = region("paste", vec![square(0.0, 0.0, 1.0, 1.0)]);

        let start = Instant::now();
        let violations = paste_overhang(
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("0.0"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "paste overhang should index sparse copper fields"
        );
    }

    #[test]
    fn paste_overhang_zero_tolerance_scales_across_many_apertures() {
        let copper = region("top", vec![square(0.0, 0.0, 20.0, 20.0)]);
        let paste = region(
            "paste",
            (0..10)
                .flat_map(|row| {
                    (0..10).map(move |column| {
                        let x = 0.25 + f64::from(column) * 1.5;
                        let y = 0.25 + f64::from(row) * 1.5;
                        square(x, y, x + 0.75, y + 0.75)
                    })
                })
                .collect(),
        );

        let start = Instant::now();
        let violations = paste_overhang(
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("0.0"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert!(violations.is_empty());
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "zero-tolerance paste overhang should not offset the same copper cover per aperture"
        );
    }

    #[test]
    fn paste_overhang_uses_retained_feature_covers() {
        let local = region("pad", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let remote = region("zone", vec![square(100.0, 100.0, 200.0, 200.0)]);
        let aggregate = region(
            "top",
            vec![
                square(0.0, 0.0, 1.0, 1.0),
                square(100.0, 100.0, 200.0, 200.0),
            ],
        );
        let paste = region("paste", vec![square(0.0, 0.0, 1.2, 1.0)]);

        let violations = paste_overhang_from_features(
            "paste",
            &paste,
            "top",
            &aggregate,
            &[&local, &remote],
            &crate::scalar::scalar("0"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "paste-aperture-overhang");
    }

    #[test]
    fn paste_aperture_coverage_reports_undersized_or_missing_apertures() {
        let copper = region(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(2.0, 0.0, 3.0, 1.0)],
        );
        let paste = region("paste", vec![square(0.1, 0.1, 0.9, 0.9)]);

        let violations = paste_aperture_coverage(
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "paste-aperture-coverage");
    }

    #[test]
    fn paste_aperture_coverage_culls_sparse_paste_fields() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let paste = region(
            "paste",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(0.0, 0.0, 0.8, 1.0)])
                .collect(),
        );

        let start = Instant::now();
        let violations = paste_aperture_coverage(
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "paste coverage should index sparse paste fields"
        );
    }

    #[test]
    fn paste_aperture_coverage_allows_full_apertures() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let paste = region("paste", vec![square(-0.1, -0.1, 1.1, 1.1)]);

        let violations = paste_aperture_coverage(
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn paste_aperture_ratio_reports_under_and_over_pasted_islands() {
        let copper = region(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(2.0, 0.0, 3.0, 1.0)],
        );
        let paste = region(
            "paste",
            vec![square(0.0, 0.0, 0.25, 1.0), square(1.9, -0.1, 3.1, 1.1)],
        );

        let violations = paste_aperture_ratio(
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("0.5"),
            &crate::scalar::scalar("1.2"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 2);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "paste-aperture-ratio")
        );
    }

    #[test]
    fn paste_aperture_ratio_allows_configured_ratio_range() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let paste = region("paste", vec![square(0.0, 0.0, 0.8, 1.0)]);

        assert!(
            paste_aperture_ratio(
                "paste",
                &paste,
                "top",
                &copper,
                &crate::scalar::scalar("0.5"),
                &crate::scalar::scalar("1.2"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn paste_aperture_ratio_culls_sparse_paste_fields() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let paste = region(
            "paste",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(0.0, 0.0, 0.25, 1.0)])
                .collect(),
        );

        let start = Instant::now();
        let violations = paste_aperture_ratio(
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("0.5"),
            &crate::scalar::scalar("1.2"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "paste aperture ratio should index sparse paste fields"
        );
    }

    #[test]
    fn minimum_paste_aperture_reports_too_narrow_apertures() {
        let paste = region("paste", vec![square(0.0, 0.0, 0.05, 0.3)]);

        let violations = minimum_paste_aperture(
            "paste",
            &paste,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "minimum-paste-aperture");
    }

    #[test]
    fn minimum_paste_aperture_allows_large_apertures() {
        let paste = region("paste", vec![square(0.0, 0.0, 0.2, 0.3)]);

        assert!(
            minimum_paste_aperture(
                "paste",
                &paste,
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn paste_aperture_spacing_reports_close_apertures() {
        let paste = region(
            "paste",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.05, 0.0, 2.05, 1.0)],
        );

        let violations = paste_aperture_spacing(
            "paste",
            &paste,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 2);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "paste-aperture-spacing")
        );
    }

    #[test]
    fn paste_aperture_spacing_allows_compliant_apertures() {
        let paste = region(
            "paste",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.2, 0.0, 2.2, 1.0)],
        );

        assert!(
            paste_aperture_spacing(
                "paste",
                &paste,
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn paste_aperture_spacing_culls_sparse_aperture_fields() {
        let mut apertures = (0..2_000)
            .map(|index| {
                let x = 100.0 + index as f64 * 3.0;
                square(x, 10.0, x + 0.5, 10.5)
            })
            .collect::<Vec<_>>();
        apertures.push(square(0.0, 0.0, 1.0, 1.0));
        apertures.push(square(1.05, 0.0, 2.05, 1.0));
        let paste = region("paste", apertures);

        let start = Instant::now();
        let violations = paste_aperture_spacing(
            "paste",
            &paste,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 2);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "paste aperture spacing should index sparse aperture fields"
        );
    }

    #[test]
    fn paste_mask_alignment_reports_paste_outside_mask_opening() {
        let paste = region("paste", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask = region("mask", vec![square(0.1, 0.0, 1.0, 1.0)]);

        let violations = paste_mask_alignment(
            "paste",
            &paste,
            "mask",
            &mask,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "paste-mask-alignment");
    }

    #[test]
    fn paste_mask_alignment_culls_sparse_mask_fields() {
        let paste = region("paste", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask = region(
            "mask",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(0.1, 0.0, 1.0, 1.0)])
                .collect(),
        );

        let start = Instant::now();
        let violations = paste_mask_alignment(
            "paste",
            &paste,
            "mask",
            &mask,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "paste-mask alignment should index sparse mask-opening fields"
        );
    }

    #[test]
    fn paste_mask_alignment_allows_paste_inside_mask_opening() {
        let paste = region("paste", vec![square(0.1, 0.1, 0.9, 0.9)]);
        let mask = region("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(
            paste_mask_alignment(
                "paste",
                &paste,
                "mask",
                &mask,
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn solder_mask_sliver_reports_thin_mask_webs() {
        let mask = region("mask", vec![square(0.0, 0.0, 0.05, 2.0)]);

        let violations = solder_mask_sliver(
            "mask",
            &mask,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn minimum_mask_opening_reports_too_small_openings() {
        let mask = region("mask", vec![square(0.0, 0.0, 0.05, 0.2)]);

        let violations = minimum_mask_opening(
            "mask",
            &mask,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "minimum-mask-opening");
    }

    #[test]
    fn minimum_mask_opening_allows_large_openings() {
        let mask = region("mask", vec![square(0.0, 0.0, 0.2, 0.2)]);

        assert!(
            minimum_mask_opening(
                "mask",
                &mask,
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn solder_mask_opening_spacing_reports_narrow_bridge() {
        let mask = region(
            "mask",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.05, 0.0, 2.05, 1.0)],
        );

        let violations = solder_mask_opening_spacing(
            "mask",
            &mask,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "solder-mask-opening-spacing")
        );
    }

    #[test]
    fn solder_mask_opening_spacing_allows_compliant_bridge() {
        let mask = region(
            "mask",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.2, 0.0, 2.2, 1.0)],
        );

        assert!(
            solder_mask_opening_spacing(
                "mask",
                &mask,
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn solder_mask_opening_spacing_culls_sparse_opening_fields() {
        let mut openings = (0..2_000)
            .map(|index| {
                let x = 100.0 + index as f64 * 3.0;
                square(x, 10.0, x + 0.5, 10.5)
            })
            .collect::<Vec<_>>();
        openings.push(square(0.0, 0.0, 1.0, 1.0));
        openings.push(square(1.05, 0.0, 2.05, 1.0));
        let mask = region("mask", openings);

        let start = Instant::now();
        let violations = solder_mask_opening_spacing(
            "mask",
            &mask,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "solder-mask opening spacing should index sparse opening fields"
        );
    }

    #[test]
    fn minimum_line_width_flags_trace_below_three_mil_threshold() {
        let three_mil_mm = 0.0762;
        let narrow_trace = region(
            "top",
            vec![line_polygon([0.0, 0.0], [1.0, 0.0], three_mil_mm * 0.8).unwrap()],
        );

        assert_eq!(
            min_copper_neck_width(
                "top",
                &narrow_trace,
                &crate::scalar::scalar("0.0762"),
                &crate::scalar::scalar("1.0e-9")
            )
            .len(),
            1
        );
    }

    #[test]
    fn minimum_line_width_allows_six_mil_preferred_trace() {
        let six_mil_mm = 0.1524;
        let preferred_trace = region(
            "top",
            vec![line_polygon([0.0, 0.0], [2.0, 0.0], six_mil_mm).unwrap()],
        );

        let violations = min_copper_neck_width(
            "top",
            &preferred_trace,
            &crate::scalar::scalar("0.0762"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert!(
            violations.is_empty(),
            "unexpected six mil trace violation area: {:?}",
            violations
                .iter()
                .map(|violation| violation.total_area)
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn convex_width_certificate_uses_exact_support_width() {
        let rectangle = square(0.0, 0.0, 2.0, 1.0);

        assert!(convex_polygon_width_at_least(
            &rectangle,
            &crate::scalar::scalar("1.0")
        ));
        assert!(!convex_polygon_width_at_least(
            &rectangle,
            &crate::scalar::scalar("1.0001")
        ));

        // Its axis-aligned bounds are both 2.0 or greater, but its exact
        // minimum support width is 4 / sqrt(5), just under 1.8.
        let diamond = Polygon::new(
            LineString(vec![
                Coord { x: 0.0, y: 1.0 },
                Coord { x: 2.0, y: 0.0 },
                Coord { x: 0.0, y: -1.0 },
                Coord { x: -2.0, y: 0.0 },
                Coord { x: 0.0, y: 1.0 },
            ]),
            vec![],
        );
        assert!(!convex_polygon_width_at_least(
            &diamond,
            &crate::scalar::scalar("1.8")
        ));
    }

    #[test]
    fn convex_width_certificate_rejects_concave_polygons() {
        let concave = Polygon::new(
            LineString(vec![
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 2.0, y: 0.0 },
                Coord { x: 1.0, y: 0.5 },
                Coord { x: 2.0, y: 1.0 },
                Coord { x: 0.0, y: 1.0 },
                Coord { x: 0.0, y: 0.0 },
            ]),
            vec![],
        );

        assert!(!convex_polygon_width_at_least(
            &concave,
            &crate::scalar::scalar("0.1")
        ));
    }

    const COMPLEX_PROJECT_FIXTURES: &[(&str, &str)] = &[
        (
            "docs/CPArti FPGA dev board.zip",
            "CPArti FPGA dev board.kicad_pcb",
        ),
        ("docs/HVP109A.zip", "HVP109A.kicad_pcb"),
    ];

    pub(crate) fn min_copper_neck_width_completes_on_complex_project_copper_layers() {
        let _geometry_guard = crate::app::COMPLEX_PROJECT_GEOMETRY_TEST_LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let mut check_elapsed = Duration::ZERO;
        let min_width = crate::scalar::scalar("0.0762");
        let min_area = crate::scalar::scalar("1.0e-9");
        let selected_layers = ["F.Cu".to_string(), "B.Cu".to_string()];

        for (zip_path, board_entry) in COMPLEX_PROJECT_FIXTURES {
            let Some(board_bytes) = unzip_fixture_entry(zip_path, board_entry) else {
                continue;
            };
            let board_path = write_temp_fixture(board_entry, &board_bytes);
            let board =
                load_kicad_pcb(&board_path).expect("complex project KiCad fixture should parse");
            let copper_layers = board.copper_layers(&selected_layers);
            let check_started = Instant::now();
            for (layer_name, copper) in copper_layers {
                let _ = min_copper_neck_width(&layer_name, &copper, &min_width, &min_area);
            }
            check_elapsed += check_started.elapsed();
            let _ = std::fs::remove_file(board_path);
        }

        assert!(
            check_elapsed < Duration::from_secs(60),
            "complex project copper neck regression fixture took {:?}",
            check_elapsed
        );
    }

    #[test]
    fn board_edge_clearance_covers_trace_below_point_two_mm() {
        let board = region("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let too_close_trace = region(
            "top",
            vec![line_polygon([0.10, 1.0], [0.10, 9.0], 0.05).unwrap()],
        );
        let compliant_trace = region(
            "top",
            vec![line_polygon([0.35, 1.0], [0.35, 9.0], 0.05).unwrap()],
        );

        assert_eq!(
            board_edge_clearance(
                "top",
                &too_close_trace,
                "edge",
                &board,
                &crate::scalar::scalar("0.20"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .len(),
            1
        );
        assert!(
            board_edge_clearance(
                "top",
                &compliant_trace,
                "edge",
                &board,
                &crate::scalar::scalar("0.20"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn board_edge_clearance_reports_pad_crossing_outline() {
        let board = region("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let pad = region("top", vec![square(9.8, 4.0, 10.2, 4.4)]);

        let violations = board_edge_clearance(
            "top",
            &pad,
            "edge",
            &board,
            &crate::scalar::scalar("0.0"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn board_outline_sanity_reports_empty_outline_layers() {
        let outline = empty_profile(Some(LayerMetadata {
            name: "edge".to_string(),
        }));

        let violations = board_outline_sanity("edge", &outline, &crate::scalar::scalar("1.0e-9"));

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn board_outline_sanity_accepts_closed_outline_area() {
        let outline = region("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);

        assert!(
            board_outline_sanity("edge", &outline, &crate::scalar::scalar("1.0e-9")).is_empty()
        );
    }

    #[test]
    fn board_outline_fragments_reports_multiple_disconnected_regions() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 1.0, 1.0), square(2.0, 0.0, 3.0, 1.0)],
        );

        let violations =
            board_outline_fragments("edge", &outline, &crate::scalar::scalar("1.0e-9"));

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-fragments");
    }

    #[test]
    fn board_outline_fragments_allows_single_region() {
        let outline = region("edge", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(
            board_outline_fragments("edge", &outline, &crate::scalar::scalar("1.0e-9")).is_empty()
        );
    }

    #[test]
    fn board_outline_self_intersection_readiness_reports_bow_tie() {
        let outline = region(
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
    fn board_outline_self_intersection_readiness_accepts_retained_gerber_grid() {
        let outline = region(
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
        let grid = SourceGridFacts::source_grid(SourceUnit::Gerber, 1_000_000);

        let violations =
            board_outline_self_intersection_readiness_with_grid("Gerber profile", &outline, grid);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "board-outline-self-intersection-readiness"
        );
    }

    #[test]
    fn board_outline_self_intersection_readiness_allows_rectangle() {
        let outline = region("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);

        assert!(board_outline_self_intersection_readiness("edge", &outline).is_empty());
    }

    #[test]
    fn board_outline_notch_readiness_reports_sharp_notch() {
        let outline = region(
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
    fn board_outline_notch_readiness_accepts_retained_gerber_grid() {
        let outline = region(
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
        let grid = SourceGridFacts::source_grid(SourceUnit::Gerber, 1_000_000);

        let violations = board_outline_notch_readiness_with_grid("Gerber profile", &outline, grid);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-notch-readiness");
    }

    #[test]
    fn board_outline_notch_readiness_allows_convex_geometry() {
        let outline = region("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);

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

        let outline = region("edge", vec![clockwise]);

        let violations = board_outline_notch_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-notch-readiness");
    }

    #[test]
    fn board_outline_notch_readiness_detects_notch_in_hole() {
        let mut outer = square(0.0, 0.0, 10.0, 10.0).exterior().0.clone();
        outer.pop();
        let outline = region(
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
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(0.0, 0.0, 10.0, 10.0)],
        );

        let violations = board_outline_duplicate_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-duplicate-readiness");
        assert!(!violations[0].locations.is_empty());
    }

    #[test]
    fn board_outline_duplicate_readiness_accepts_retained_gerber_grid() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(0.0, 0.0, 10.0, 10.0)],
        );
        let grid = SourceGridFacts::source_grid(SourceUnit::Gerber, 1_000_000);

        let violations =
            board_outline_duplicate_readiness_with_grid("Gerber profile", &outline, grid);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-duplicate-readiness");
    }

    #[test]
    fn board_outline_duplicate_readiness_allows_discrete_outer_regions() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(20.0, 0.0, 30.0, 10.0)],
        );

        assert!(board_outline_duplicate_readiness("edge", &outline).is_empty());
    }

    #[test]
    fn board_outline_nesting_readiness_reports_nested_contour() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(2.0, 2.0, 4.0, 4.0)],
        );

        let violations = board_outline_nesting_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-nesting-readiness");
        assert!(!violations[0].locations.is_empty());
    }

    #[test]
    fn board_outline_nesting_readiness_accepts_retained_gerber_grid() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(2.0, 2.0, 4.0, 4.0)],
        );
        let grid = SourceGridFacts::source_grid(SourceUnit::Gerber, 1_000_000);

        let violations =
            board_outline_nesting_readiness_with_grid("Gerber profile", &outline, grid);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-nesting-readiness");
    }

    #[test]
    fn board_outline_nesting_readiness_allows_non_nested_discrete_regions() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(20.0, 0.0, 30.0, 10.0)],
        );

        assert!(board_outline_nesting_readiness("edge", &outline).is_empty());
    }

    #[test]
    fn board_outline_nesting_readiness_allows_touching_contours() {
        let outline = region(
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

        let outline = region("edge", vec![square(0.0, 0.0, 10.0, 10.0), duplicate]);

        let violations = board_outline_duplicate_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-duplicate-readiness");
    }

    #[test]
    fn board_outline_cutout_clearance_reports_nested_inner_region_intrusion() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(3.0, 3.0, 7.0, 7.0)],
        );
        let subject = region("top", vec![square(4.0, 4.0, 6.0, 6.0)]);

        let violations = board_outline_cutout_clearance(
            "top",
            &subject,
            "edge",
            &outline,
            &crate::scalar::scalar("0"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-cutout-clearance");
    }

    #[test]
    fn board_outline_cutout_clearance_reports_nearby_geometry_with_clearance() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(3.0, 3.0, 7.0, 7.0)],
        );
        let near = region("top", vec![square(7.15, 4.0, 7.45, 6.0)]);

        let violations = board_outline_cutout_clearance(
            "top",
            &near,
            "edge",
            &outline,
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-cutout-clearance");
    }

    #[test]
    fn board_outline_cutout_clearance_allows_geometry_outside_clearance_band() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(3.0, 3.0, 7.0, 7.0)],
        );
        let far = region("top", vec![square(7.8, 4.0, 8.2, 6.0)]);

        let violations = board_outline_cutout_clearance(
            "top",
            &far,
            "edge",
            &outline,
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn board_outline_cutout_clearance_allows_geometry_outside_cutout_region() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(3.0, 3.0, 7.0, 7.0)],
        );
        let subject = region("top", vec![square(1.0, 1.0, 2.0, 2.0)]);

        let violations = board_outline_cutout_clearance(
            "top",
            &subject,
            "edge",
            &outline,
            &crate::scalar::scalar("0"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn board_outline_cutout_clearance_allows_non_nested_outline_regions() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(12.0, 0.0, 15.0, 2.0)],
        );
        let subject = region("top", vec![square(1.0, 1.0, 2.0, 2.0)]);

        assert!(
            board_outline_cutout_clearance(
                "top",
                &subject,
                "edge",
                &outline,
                &crate::scalar::scalar("0"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn board_outline_cutout_clearance_reports_multiple_nested_regions() {
        let outline = region(
            "edge",
            vec![
                square(0.0, 0.0, 20.0, 20.0),
                square(3.0, 3.0, 5.0, 5.0),
                square(12.0, 12.0, 14.0, 14.0),
            ],
        );
        let subject = region(
            "top",
            vec![square(4.0, 4.0, 4.5, 4.5), square(13.0, 13.0, 13.5, 13.5)],
        );

        let violations = board_outline_cutout_clearance(
            "top",
            &subject,
            "edge",
            &outline,
            &crate::scalar::scalar("0"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 2);
    }

    #[test]
    fn board_outline_cutout_clearance_flags_zero_clearance_touching_geometry() {
        let outline = region(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(3.0, 3.0, 7.0, 7.0)],
        );
        let touching = region("top", vec![square(2.0, 4.0, 3.0, 6.0)]);

        let violations = board_outline_cutout_clearance(
            "top",
            &touching,
            "edge",
            &outline,
            &crate::scalar::scalar("0"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn board_outline_cutout_clearance_is_orientation_tolerant_for_cutouts() {
        let mut inner = square(3.0, 3.0, 7.0, 7.0).exterior().0.clone();
        inner.pop();
        inner.reverse();
        inner.push(inner[0]);
        let outline = region(
            "edge",
            vec![
                square(0.0, 0.0, 10.0, 10.0),
                Polygon::new(LineString(inner), vec![]),
            ],
        );
        let near = region("top", vec![square(7.15, 4.0, 7.45, 6.0)]);

        let violations = board_outline_cutout_clearance(
            "top",
            &near,
            "edge",
            &outline,
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn exposed_copper_reports_oversized_mask_opening_touching_neighbor() {
        let copper = region(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.2, 0.0, 2.2, 1.0)],
        );
        let mask_opening = region("mask", vec![square(-0.1, -0.1, 1.35, 1.1)]);

        let violations = exposed_copper(
            "top",
            &copper,
            "mask",
            &mask_opening,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn exposed_copper_culls_sparse_mask_fields() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_opening = region(
            "mask",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(0.2, 0.2, 0.8, 0.8)])
                .collect(),
        );

        let start = Instant::now();
        let violations = exposed_copper(
            "top",
            &copper,
            "mask",
            &mask_opening,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "exposed copper should index sparse mask-opening fields"
        );
    }

    #[test]
    fn solder_mask_opening_coverage_reports_undersized_or_missing_openings() {
        let copper = region(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(2.0, 0.0, 3.0, 1.0)],
        );
        let mask_openings = region("mask", vec![square(0.1, 0.1, 0.9, 0.9)]);

        let violations = solder_mask_opening_coverage(
            "top",
            &copper,
            "mask",
            &mask_openings,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-opening-coverage");
    }

    #[test]
    fn solder_mask_opening_coverage_allows_full_openings() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_openings = region("mask", vec![square(-0.1, -0.1, 1.1, 1.1)]);

        let violations = solder_mask_opening_coverage(
            "top",
            &copper,
            "mask",
            &mask_openings,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn solder_mask_opening_ratio_reports_under_and_over_openings() {
        let copper = region(
            "top",
            vec![
                square(0.0, 0.0, 1.0, 1.0),
                square(3.0, 0.0, 4.0, 1.0),
                square(6.0, 0.0, 7.0, 1.0),
            ],
        );
        let mask_openings = region(
            "mask",
            vec![
                square(0.1, 0.1, 0.9, 0.9),
                square(2.95, -0.05, 4.05, 1.05),
                square(5.5, -0.5, 7.5, 1.5),
            ],
        );

        let violations = solder_mask_opening_ratio_readiness(
            "top",
            &copper,
            "mask",
            &mask_openings,
            &crate::scalar::scalar("1.0"),
            &crate::scalar::scalar("1.5"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 2);
        assert!(violations.iter().all(|violation| {
            violation.check == "solder-mask-opening-ratio-readiness"
                && violation
                    .message
                    .as_deref()
                    .is_some_and(|message| message.contains("NSMD/SMD"))
        }));
    }

    #[test]
    fn solder_mask_opening_ratio_allows_configured_range() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_openings = region("mask", vec![square(-0.05, -0.05, 1.05, 1.05)]);

        assert!(
            solder_mask_opening_ratio_readiness(
                "top",
                &copper,
                "mask",
                &mask_openings,
                &crate::scalar::scalar("1.0"),
                &crate::scalar::scalar("1.5"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn solder_mask_opening_ratio_culls_sparse_mask_fields() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_openings = region(
            "mask",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 1.1, 11.1)
                })
                .chain([square(-0.05, -0.05, 1.05, 1.05)])
                .collect(),
        );

        let start = Instant::now();
        let violations = solder_mask_opening_ratio_readiness(
            "top",
            &copper,
            "mask",
            &mask_openings,
            &crate::scalar::scalar("1.0"),
            &crate::scalar::scalar("1.5"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert!(violations.is_empty());
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "solder-mask opening-ratio review should index sparse opening fields"
        );
    }

    #[test]
    fn solder_mask_annular_ring_reports_tight_openings() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let tight_mask = region("mask", vec![square(-0.03, -0.03, 1.03, 1.03)]);

        let violations = solder_mask_annular_ring_readiness(
            "top",
            &copper,
            "mask",
            &tight_mask,
            &crate::scalar::scalar("0.08"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-annular-ring-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("minimum mask annular ring"))
        );
    }

    #[test]
    fn solder_mask_annular_ring_allows_configured_relief() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask = region("mask", vec![square(-0.10, -0.10, 1.10, 1.10)]);

        assert!(
            solder_mask_annular_ring_readiness(
                "top",
                &copper,
                "mask",
                &mask,
                &crate::scalar::scalar("0.08"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn solder_mask_annular_ring_culls_sparse_mask_opening_fields() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask = region(
            "mask",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(-0.03, -0.03, 1.03, 1.03)])
                .collect(),
        );

        let start = Instant::now();
        let violations = solder_mask_annular_ring_readiness(
            "top",
            &copper,
            "mask",
            &mask,
            &crate::scalar::scalar("0.08"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "solder-mask annular-ring check should index sparse opening fields"
        );
    }

    #[test]
    fn solder_mask_opening_coverage_culls_sparse_mask_fields() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_openings = region(
            "mask",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(0.0, 0.0, 0.8, 1.0)])
                .collect(),
        );

        let start = Instant::now();
        let violations = solder_mask_opening_coverage(
            "top",
            &copper,
            "mask",
            &mask_openings,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "solder-mask coverage should index sparse opening fields"
        );
    }

    #[test]
    fn solder_mask_expansion_reports_oversized_opening() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_openings = region("mask", vec![square(-0.2, -0.2, 1.2, 1.2)]);

        let violations = solder_mask_expansion(
            "top",
            &copper,
            "mask",
            &mask_openings,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-expansion");
    }

    #[test]
    fn solder_mask_expansion_uses_retained_feature_covers() {
        let local = region("pad", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let remote = region("zone", vec![square(100.0, 100.0, 200.0, 200.0)]);
        let aggregate = region(
            "top",
            vec![
                square(0.0, 0.0, 1.0, 1.0),
                square(100.0, 100.0, 200.0, 200.0),
            ],
        );
        let mask_openings = region("mask", vec![square(-0.2, -0.2, 1.2, 1.2)]);

        let violations = solder_mask_expansion_from_features(
            "top",
            &aggregate,
            &[&local, &remote],
            "mask",
            &mask_openings,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-expansion");
    }

    #[test]
    fn solder_mask_feature_difference_unions_exact_component_remainders() {
        let left = region("pad-left", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let right = region("pad-right", vec![square(4.0, 0.0, 5.0, 1.0)]);
        let aggregate = region(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(4.0, 0.0, 5.0, 1.0)],
        );
        let mask_openings = region(
            "mask",
            vec![square(-0.2, -0.2, 1.2, 1.2), square(3.8, -0.2, 5.2, 1.2)],
        );

        let violations = solder_mask_expansion_from_features(
            "top",
            &aggregate,
            &[&left, &right],
            "mask",
            &mask_openings,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert!(!violations.is_empty());
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "solder-mask-expansion"),
            "exact component reassembly must not fall back to geometry uncertainty: {violations:?}"
        );
    }

    #[test]
    fn solder_mask_expansion_culls_sparse_copper_fields() {
        let copper = region(
            "top",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(0.0, 0.0, 1.0, 1.0)])
                .collect(),
        );
        let mask_openings = region("mask", vec![square(-0.2, -0.2, 1.2, 1.2)]);

        let start = Instant::now();
        let violations = solder_mask_expansion(
            "top",
            &copper,
            "mask",
            &mask_openings,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "solder-mask expansion should index sparse copper fields"
        );
    }

    #[test]
    fn solder_mask_expansion_allows_configured_opening_growth() {
        let copper = region("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_openings = region("mask", vec![square(-0.05, -0.05, 1.05, 1.05)]);

        assert!(
            solder_mask_expansion(
                "top",
                &copper,
                "mask",
                &mask_openings,
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn solder_mask_overlap_clearance_reports_adjacent_covered_copper() {
        let copper = region("top", vec![square(1.05, 0.0, 1.20, 1.0)]);
        let mask_openings = region("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);

        let violations = solder_mask_overlap_clearance(
            "top",
            &copper,
            "mask",
            &mask_openings,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-overlap-clearance");
    }

    #[test]
    fn solder_mask_overlap_clearance_ignores_intentionally_open_copper() {
        let copper = region("top", vec![square(0.1, 0.1, 0.9, 0.9)]);
        let mask_openings = region("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(
            solder_mask_overlap_clearance(
                "top",
                &copper,
                "mask",
                &mask_openings,
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn solder_mask_overlap_clearance_allows_distant_covered_copper() {
        let copper = region("top", vec![square(1.2, 0.0, 1.4, 1.0)]);
        let mask_openings = region("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(
            solder_mask_overlap_clearance(
                "top",
                &copper,
                "mask",
                &mask_openings,
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn solder_mask_overlap_clearance_culls_sparse_opening_fields() {
        let copper = region("top", vec![square(1.05, 0.0, 1.20, 1.0)]);
        let mask_openings = region(
            "mask",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(0.0, 0.0, 1.0, 1.0)])
                .collect(),
        );

        let start = Instant::now();
        let violations = solder_mask_overlap_clearance(
            "top",
            &copper,
            "mask",
            &mask_openings,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "solder-mask overlap clearance should index sparse opening fields"
        );
    }

    #[test]
    fn silkscreen_overlap_reports_legend_over_pad_or_slot() {
        let pad_opening = region("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let silk_text_stroke = region(
            "silk",
            vec![line_polygon([-0.2, 0.5], [1.2, 0.5], 0.08).unwrap()],
        );

        let violations = silkscreen_overlap(
            "silk",
            &silk_text_stroke,
            "mask",
            &pad_opening,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn silkscreen_overlap_reports_legend_over_v_score_or_slot_geometry() {
        let panel_feature = region(
            "V-Score",
            vec![line_polygon([0.5, -1.0], [0.5, 1.0], 0.12).unwrap()],
        );
        let silk_text_stroke = region(
            "B.SilkS",
            vec![line_polygon([0.0, 0.0], [1.0, 0.0], 0.08).unwrap()],
        );

        let violations = silkscreen_overlap(
            "B.SilkS",
            &silk_text_stroke,
            "V-Score",
            &panel_feature,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "silkscreen-overlap");
    }

    #[test]
    fn silkscreen_overlap_culls_sparse_blocker_fields() {
        let blockers = region(
            "mask",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(0.0, 0.0, 1.0, 1.0)])
                .collect(),
        );
        let silk_text_stroke = region(
            "silk",
            vec![line_polygon([-0.2, 0.5], [1.2, 0.5], 0.08).unwrap()],
        );

        let start = Instant::now();
        let violations = silkscreen_overlap(
            "silk",
            &silk_text_stroke,
            "mask",
            &blockers,
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "silkscreen overlap should index sparse blocker fields"
        );
    }

    #[test]
    fn silkscreen_clearance_reports_legend_near_blocker() {
        let pad_opening = region("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let silk_text_stroke = region(
            "silk",
            vec![line_polygon([1.08, 0.5], [1.8, 0.5], 0.05).unwrap()],
        );

        let violations = silkscreen_clearance(
            "silk",
            &silk_text_stroke,
            "mask",
            &pad_opening,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "silkscreen-clearance");
    }

    #[test]
    fn silkscreen_clearance_allows_distant_legend() {
        let pad_opening = region("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let silk_text_stroke = region(
            "silk",
            vec![line_polygon([1.3, 0.5], [1.8, 0.5], 0.05).unwrap()],
        );

        assert!(
            silkscreen_clearance(
                "silk",
                &silk_text_stroke,
                "mask",
                &pad_opening,
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn silkscreen_clearance_culls_sparse_blocker_fields() {
        let blockers = region(
            "mask",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    square(x, 10.0, x + 0.5, 10.5)
                })
                .chain([square(0.0, 0.0, 1.0, 1.0)])
                .collect(),
        );
        let silk_text_stroke = region(
            "silk",
            vec![line_polygon([1.08, 0.5], [1.8, 0.5], 0.05).unwrap()],
        );

        let start = Instant::now();
        let violations = silkscreen_clearance(
            "silk",
            &silk_text_stroke,
            "mask",
            &blockers,
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "silkscreen clearance should index sparse blocker fields"
        );
    }

    #[test]
    fn silkscreen_min_width_reports_thin_legend_strokes() {
        let silk = region(
            "silk",
            vec![line_polygon([0.0, 0.0], [2.0, 0.0], 0.08).unwrap()],
        );

        let violations = silkscreen_min_width(
            "silk",
            &silk,
            &crate::scalar::scalar("0.12"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn silkscreen_min_width_allows_wide_legend_strokes() {
        let silk = region("silk", vec![square(0.0, 0.0, 1.0, 1.0)]);

        let violations = silkscreen_min_width(
            "silk",
            &silk,
            &crate::scalar::scalar("0.12"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn silkscreen_text_height_reports_tiny_legend_islands() {
        let silk = region("silk", vec![square(0.0, 0.0, 0.45, 0.55)]);

        let violations = silkscreen_text_height_readiness(
            "silk",
            &silk,
            &crate::scalar::scalar("0.80"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "silkscreen-text-height-readiness");
        assert_eq!(violations[0].locations.len(), 1);
    }

    #[test]
    fn silkscreen_text_height_allows_tall_legend_islands_and_long_lines() {
        let silk = region(
            "silk",
            vec![
                square(0.0, 0.0, 0.60, 0.90),
                line_polygon([2.0, 0.0], [3.2, 0.0], 0.08).unwrap(),
            ],
        );

        assert!(
            silkscreen_text_height_readiness(
                "silk",
                &silk,
                &crate::scalar::scalar("0.80"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn layer_sanity_reports_empty_or_unbounded_layers() {
        let empty = empty_profile(Some(LayerMetadata {
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
        let bad_outline = region(
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
        let bad_outline = region(
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
        let invalid = region(
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
        let invalid = region(
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
        let flood = region("inner", vec![square(0.0, 0.0, 20.0, 20.0)]);

        let maximum = crate::scalar::scalar("100");
        let violations = layer_sanity("inner", &flood, Some(&maximum));

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
        let flood = region("inner", vec![square(0.0, 0.0, 10.0, 10.0)]);

        let maximum = crate::scalar::scalar("100");
        let violations = layer_sanity("inner", &flood, Some(&maximum));

        assert!(violations.iter().all(|violation| {
            violation
                .message
                .as_deref()
                .is_none_or(|message| !message.contains("exceeds maximum"))
        }));
    }

    #[test]
    fn duplicate_layer_geometry_readiness_reports_identical_layers() {
        let top = region("F.Cu", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let duplicate = region("B.Cu", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let layers = vec![("F.Cu".to_string(), top), ("B.Cu".to_string(), duplicate)];

        let violations =
            duplicate_layer_geometry_readiness(&layers, &crate::scalar::scalar("1.0e-9"));

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
        let top = region("F.Cu", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let shifted = region("B.Cu", vec![square(12.0, 0.0, 22.0, 10.0)]);
        let empty = empty_profile(Some(LayerMetadata {
            name: "empty".to_string(),
        }));
        let layers = vec![
            ("F.Cu".to_string(), top),
            ("B.Cu".to_string(), shifted),
            ("empty".to_string(), empty),
        ];

        assert!(
            duplicate_layer_geometry_readiness(&layers, &crate::scalar::scalar("1.0e-9"))
                .is_empty()
        );
    }

    #[test]
    fn tiny_layer_feature_readiness_reports_islands_below_area_gate() {
        let layer = region(
            "F.Cu",
            vec![
                square(0.0, 0.0, 10.0, 10.0),
                square(20.0, 20.0, 20.05, 20.05),
            ],
        );

        let violations =
            tiny_layer_feature_readiness("F.Cu", &layer, &crate::scalar::scalar("0.01"));

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
        let layer = region("F.Cu", vec![square(0.0, 0.0, 10.0, 10.0)]);

        assert!(
            tiny_layer_feature_readiness("F.Cu", &layer, &crate::scalar::scalar("0.01")).is_empty()
        );
        assert!(
            tiny_layer_feature_readiness("F.Cu", &layer, &crate::scalar::scalar("0")).is_empty()
        );
        assert!(
            tiny_layer_feature_readiness("F.Cu", &layer, &crate::scalar::scalar("-1")).is_empty()
        );
    }

    #[test]
    fn skinny_layer_feature_readiness_reports_long_slivers_above_area_gate() {
        let layer = region(
            "F.Cu",
            vec![
                square(0.0, 0.0, 10.0, 10.0),
                rect_polygon([20.0, 20.0], [4.0, 0.05], 0.0),
            ],
        );

        let violations = skinny_layer_feature_readiness(
            "F.Cu",
            &layer,
            &crate::scalar::scalar("0.10"),
            &crate::scalar::scalar("0.01"),
        );

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
        let wide = region("F.Cu", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let tiny = region("F.Cu", vec![rect_polygon([0.0, 0.0], [0.09, 0.09], 0.0)]);

        for width in ["0.10", "0", "-1"] {
            assert!(
                skinny_layer_feature_readiness(
                    "F.Cu",
                    &wide,
                    &crate::scalar::scalar(width),
                    &crate::scalar::scalar("0.01"),
                )
                .is_empty()
            );
        }
        assert!(
            skinny_layer_feature_readiness(
                "F.Cu",
                &tiny,
                &crate::scalar::scalar("0.10"),
                &crate::scalar::scalar("0.01"),
            )
            .is_empty()
        );
    }

    #[test]
    fn duplicate_layer_island_readiness_reports_repeated_polygon_geometry() {
        let duplicate = square(0.0, 0.0, 10.0, 10.0);
        let layer = region("F.Cu", vec![duplicate.clone(), duplicate]);

        let violations =
            duplicate_layer_island_readiness("F.Cu", &layer, &crate::scalar::scalar("1.0e-9"));

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
        let discrete = region(
            "F.Cu",
            vec![square(0.0, 0.0, 10.0, 10.0), square(12.0, 0.0, 22.0, 10.0)],
        );
        let tiny_duplicate = rect_polygon([0.0, 0.0], [0.03, 0.03], 0.0);
        let tiny = region("F.Cu", vec![tiny_duplicate.clone(), tiny_duplicate]);

        assert!(
            duplicate_layer_island_readiness("F.Cu", &discrete, &crate::scalar::scalar("1.0e-9"),)
                .is_empty()
        );
        assert!(
            duplicate_layer_island_readiness("F.Cu", &tiny, &crate::scalar::scalar("0.01"),)
                .is_empty()
        );
    }

    #[test]
    fn board_outline_self_intersection_readiness_reports_hole_self_intersection() {
        let outline = region(
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

        assert!(
            !super::polygon_contains_other_outer(
                &outer,
                &touching,
                &crate::scalar::scalar(super::BOARD_OUTLINE_NESTED_OVERLAP_RATIO),
                &crate::scalar::scalar(super::BOARD_OUTLINE_GEOMETRY_TOLERANCE),
                "test",
                vec!["edge".to_string()],
            )
            .expect("test geometry should certify")
        );
        assert!(
            super::polygons_are_duplicate(
                &outer,
                &outer,
                &crate::scalar::scalar(super::BOARD_OUTLINE_GEOMETRY_TOLERANCE),
                "test",
                vec!["edge".to_string()],
            )
            .expect("test geometry should certify")
        );
    }

    #[test]
    fn mechanical_layer_geometry_reports_shapes_on_user_or_mechanical_layers() {
        let user = region("Dwgs.User", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mechanical = region("board-Mechanical.gbr", vec![square(2.0, 0.0, 3.0, 1.0)]);

        assert_eq!(
            mechanical_layer_geometry("Dwgs.User", &user, &crate::scalar::scalar("1.0e-9")).len(),
            1
        );
        assert_eq!(
            mechanical_layer_geometry(
                "board-Mechanical.gbr",
                &mechanical,
                &crate::scalar::scalar("1.0e-9"),
            )
            .len(),
            1
        );
    }

    #[test]
    fn mechanical_layer_geometry_ignores_normal_copper_layers() {
        let copper = region("F.Cu", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(
            mechanical_layer_geometry("F.Cu", &copper, &crate::scalar::scalar("1.0e-9")).is_empty()
        );
    }

    #[test]
    fn acid_trap_reports_acute_polygon_vertices() {
        let copper = region(
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

        let violations = acid_trap_candidates("top", &copper, &crate::scalar::scalar("30.0"));

        assert_eq!(violations.len(), 1);
        assert!(!violations[0].locations.is_empty());
    }

    fn region(name: &str, polygons: Vec<Polygon<f64>>) -> crate::PcbRegion {
        polygons_to_profile(
            polygons,
            Some(LayerMetadata {
                name: name.to_string(),
            }),
        )
    }

    fn ipc_point(net: &str, location: [f64; 2]) -> Ipc356Point {
        Ipc356Point {
            net: net.to_string(),
            reference: None,
            pin: None,
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            diameter: None,
            access_side: None,
            feature_type: None,
            soldermask: None,
        }
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
