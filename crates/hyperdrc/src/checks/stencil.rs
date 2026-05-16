//! Stencil and solder-paste readiness checks.
//!
//! These checks live apart from generic layer and board checks because paste
//! review often combines flattened Gerber aperture geometry with richer KiCad
//! via or drill context.

use csgrs::csg::CSG;
use geo::{Area, BoundingRect, Polygon};

use crate::geometry::{
    circle_polygon, multipolygon_to_shapes, polygon_to_sketch, polygons_to_sketch,
};
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch};

use super::spatial::{LayerPolygonSpatialIndex, PointSpatialIndex};

/// Warn when a large copper island is pasted as one broad aperture.
///
/// IPC-7525B frames stencil aperture design around paste release and volume
/// control; for bottom-termination thermal pads that usually means reduced or
/// split apertures rather than full pad coverage. QFN voiding studies also tie
/// thermal pad solder geometry and vias to voiding risk; see Wilcoxon, Pearson,
/// and Hillman, "Modeling the Effects of Thermal Pad Voiding on Quad Flatpack
/// No-lead (QFN) Components," Journal of SMT, 2023, doi:10.37665/smt.v36i2.37.
/// Paste apertures are selected through the shared layer-polygon broad phase
/// before exact copper/paste intersection, following Ericson, *Real-Time
/// Collision Detection* (2005), so sparse stencil exports do not force every
/// thermal pad candidate to scan every aperture.
pub fn thermal_pad_paste_windowpane_readiness(
    paste_name: &str,
    paste: &PcbSketch,
    copper_name: &str,
    copper: &PcbSketch,
    min_copper_area: f64,
    max_single_aperture_ratio: f64,
    min_area: f64,
) -> Vec<Violation> {
    let paste_polygons = paste.to_multipolygon().0;
    let paste_index = LayerPolygonSpatialIndex::new(&paste_polygons, 0.0);
    let mut violations = Vec::new();
    let mut candidate_apertures = 0usize;

    for (island_index, copper_polygon) in copper.to_multipolygon().0.into_iter().enumerate() {
        let copper_area = copper_polygon.unsigned_area();
        if copper_area < min_copper_area {
            continue;
        }

        let paste_candidates = paste_index.candidates_near_polygon(&copper_polygon, 0.0);
        candidate_apertures += paste_candidates.len();
        let island = polygon_to_sketch(copper_polygon, Some(metadata(copper_name)));
        let mut intersecting_apertures = 0usize;
        let mut paste_area = 0.0;
        for paste_index in paste_candidates {
            let paste_island = polygon_to_sketch(
                paste_polygons[paste_index].clone(),
                Some(metadata(paste_name)),
            );
            let overlap = island.intersection(&paste_island).to_multipolygon();
            let overlap_area = overlap.unsigned_area();
            if overlap_area <= min_area {
                continue;
            }
            intersecting_apertures += 1;
            paste_area += overlap_area;
        }

        let ratio = paste_area / copper_area;
        if intersecting_apertures >= 2 || ratio <= max_single_aperture_ratio {
            continue;
        }

        violations.push(Violation::new(
            "thermal-pad-paste-windowpane-readiness",
            Severity::Warning,
            vec![paste_name.to_string(), copper_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes(&island.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "large copper island has one paste aperture with ratio {ratio:.3}; review windowpane paste reduction for thermal pad solder voiding"
            )),
        ));
    }

    log::trace!(
        "thermal pad paste windowpane readiness: paste={} copper={} paste_apertures={} paste_buckets={} candidate_apertures={} violations={}",
        paste_name,
        copper_name,
        paste_polygons.len(),
        paste_index.bucket_count(),
        candidate_apertures,
        violations.len()
    );

    violations
}

/// Warn on apertures whose opening area is too small for their wall area.
///
/// IPC-7525B defines area ratio as aperture opening area divided by aperture
/// wall area and uses it as a paste-transfer metric. Harter et al., "The Effect
/// of Area Shape and Area Ratio on Solder Paste Printing Performance," SMTAI
/// 2016, experimentally studied how area ratio and aperture shape affect print
/// transfer efficiency. HyperDRC estimates the wall area from each aperture's
/// bounding rectangle until exact stencil thickness and aperture side-wall
/// models become available.
pub fn stencil_area_ratio_readiness(
    paste_name: &str,
    paste: &PcbSketch,
    stencil_thickness: f64,
    min_area_ratio: f64,
    min_area: f64,
) -> Vec<Violation> {
    if stencil_thickness <= 0.0 {
        return Vec::new();
    }

    let mut violations = Vec::new();
    for (island_index, polygon) in paste.to_multipolygon().0.into_iter().enumerate() {
        let aperture_area = polygon.unsigned_area();
        if aperture_area <= min_area {
            continue;
        }
        let Some(bounds) = polygon.bounding_rect() else {
            continue;
        };
        let width = bounds.max().x - bounds.min().x;
        let height = bounds.max().y - bounds.min().y;
        if width <= 0.0 || height <= 0.0 {
            continue;
        }

        let wall_area = 2.0 * (width + height) * stencil_thickness;
        if wall_area <= 0.0 {
            continue;
        }
        let area_ratio = aperture_area / wall_area;
        if area_ratio >= min_area_ratio {
            continue;
        }

        let aperture = polygon_to_sketch(polygon, Some(metadata(paste_name)));
        violations.push(Violation::new(
            "stencil-area-ratio-readiness",
            Severity::Warning,
            vec![paste_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes(&aperture.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "stencil aperture area ratio {area_ratio:.3} is below minimum {min_area_ratio:.3}; review stencil thickness, aperture size, or paste release process"
            )),
        ));
    }

    violations
}

/// Warn on long, narrow paste apertures that may release or slump poorly.
///
/// IPC-7525B defines stencil aspect and area ratios as first-order aperture
/// printability metrics. This geometry-only check uses bounding-box elongation
/// as a conservative proxy until explicit stencil thickness is modeled.
pub fn paste_aperture_aspect_ratio_readiness(
    paste_name: &str,
    paste: &PcbSketch,
    max_aspect_ratio: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for (island_index, polygon) in paste.to_multipolygon().0.into_iter().enumerate() {
        if polygon.unsigned_area() <= min_area {
            continue;
        }
        let Some(bounds) = polygon.bounding_rect() else {
            continue;
        };
        let width = bounds.max().x - bounds.min().x;
        let height = bounds.max().y - bounds.min().y;
        let min_dimension = width.min(height);
        let max_dimension = width.max(height);
        if min_dimension <= 0.0 {
            continue;
        }
        let aspect_ratio = max_dimension / min_dimension;
        if aspect_ratio <= max_aspect_ratio {
            continue;
        }

        let aperture = polygon_to_sketch(polygon, Some(metadata(paste_name)));
        violations.push(Violation::new(
            "paste-aperture-aspect-ratio-readiness",
            Severity::Warning,
            vec![paste_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes(&aperture.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "paste aperture aspect ratio {aspect_ratio:.3} exceeds {max_aspect_ratio:.3}; review stencil release and slumping risk"
            )),
        ));
    }

    violations
}

/// Warn when neighboring small pads receive very different paste coverage.
///
/// Without footprint metadata this treats close, similarly sized copper islands
/// as likely two-terminal pad pairs and compares their paste-to-copper ratios.
/// The heuristic follows the manufacturing idea, documented in IPC-7525B and
/// chip-component tombstoning literature, that unbalanced wetting and thermal
/// conditions across the two terminations increase tombstoning risk. Candidate
/// pad pairs are selected with a deterministic point-grid broad phase before
/// area-ratio and paste-ratio checks, following Ericson, *Real-Time Collision
/// Detection* (2005). Paste coverage for each pad uses the same broad-phase
/// idea over aperture polygons before exact copper/paste intersection, so large
/// sparse stencil exports do not force every pad to intersect every aperture.
pub fn tombstone_paste_imbalance_readiness(
    paste_name: &str,
    paste: &PcbSketch,
    copper_name: &str,
    copper: &PcbSketch,
    max_pair_gap: f64,
    max_ratio_delta: f64,
    min_area: f64,
) -> Vec<Violation> {
    let copper_polygons = copper.to_multipolygon().0;
    let paste_polygons = paste.to_multipolygon().0;
    let paste_index = LayerPolygonSpatialIndex::new(&paste_polygons, 0.0);
    let mut islands = Vec::new();
    let mut paste_candidate_polygons = 0usize;
    for (index, polygon) in copper_polygons.into_iter().enumerate() {
        let area = polygon.unsigned_area();
        if area <= min_area {
            continue;
        }
        let Some(center) = polygon_center(&polygon) else {
            continue;
        };
        let paste_candidates = paste_index.candidates_near_polygon(&polygon, 0.0);
        paste_candidate_polygons += paste_candidates.len();
        let island = polygon_to_sketch(polygon, Some(metadata(copper_name)));
        let paste_area = paste_candidates
            .into_iter()
            .map(|paste_index| {
                let paste_island = polygon_to_sketch(
                    paste_polygons[paste_index].clone(),
                    Some(metadata(paste_name)),
                );
                island
                    .intersection(&paste_island)
                    .to_multipolygon()
                    .unsigned_area()
            })
            .sum::<f64>();
        islands.push((index, island, center, area, paste_area / area));
    }

    let center_index = PointSpatialIndex::new(
        islands.iter().map(|(_, _, center, _, _)| *center),
        max_pair_gap,
    );
    let mut candidate_pairs = 0_usize;
    let mut violations = Vec::new();
    for left_index in 0..islands.len() {
        let (left_original_index, left_island, left_center, left_area, left_ratio) =
            &islands[left_index];
        for right_index in center_index.centers_within(*left_center, max_pair_gap) {
            if right_index <= left_index {
                continue;
            }
            candidate_pairs += 1;
            let (right_original_index, right_island, _right_center, right_area, right_ratio) =
                &islands[right_index];
            let area_ratio = left_area.max(*right_area) / left_area.min(*right_area);
            if area_ratio > 1.5 {
                continue;
            }
            let delta = (left_ratio - right_ratio).abs();
            if delta <= max_ratio_delta {
                continue;
            }

            let combined = left_island.union(right_island);
            violations.push(Violation::new(
                "tombstone-paste-imbalance-readiness",
                Severity::Warning,
                vec![paste_name.to_string(), copper_name.to_string()],
                Some(*left_original_index.min(right_original_index)),
                multipolygon_to_shapes(&combined.to_multipolygon(), min_area),
                Vec::new(),
                Some(format!(
                    "neighboring small copper islands have paste ratio imbalance {delta:.3}; review tombstoning risk on two-terminal components"
                )),
            ));
        }
    }
    log::trace!(
        "tombstone paste imbalance readiness: paste={} copper={} islands={} paste_buckets={} paste_candidate_polygons={} pair_buckets={} candidate_pairs={} max_pair_gap={max_pair_gap:.6} violations={}",
        paste_name,
        copper_name,
        islands.len(),
        paste_index.bucket_count(),
        paste_candidate_polygons,
        center_index.bucket_count(),
        candidate_pairs,
        violations.len()
    );

    violations
}

/// Warn when explicit paste apertures cover parsed via drill openings.
///
/// Paste on via openings can wick solder away from pads unless the via is
/// filled, capped, or intentionally tented. IPC-7525B and QFN via-design
/// studies call out via placement under pasted thermal pads as an assembly
/// variable because it can affect voiding and solder protrusion.
pub fn paste_via_exposure_readiness(
    paste_name: &str,
    paste: &PcbSketch,
    board: &BoardModel,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let vias = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Via)
        .collect::<Vec<_>>();
    let paste_polygons = paste.to_multipolygon().0;
    let paste_index = LayerPolygonSpatialIndex::new(&paste_polygons, 0.0);
    let mut violations = Vec::new();
    let mut candidate_apertures = 0usize;

    for via in &vias {
        let via_opening = matching_plated_drill(board, via)
            .map(|drill| {
                polygons_to_sketch(
                    vec![circle_polygon(drill.location, drill.diameter / 2.0, 48)],
                    Some(LayerMetadata {
                        name: "via drill opening".to_string(),
                    }),
                )
            })
            .unwrap_or_else(|| via.sketch.clone());
        let via_polygons = via_opening.to_multipolygon().0;
        let paste_candidates = via_polygons
            .iter()
            .flat_map(|polygon| paste_index.candidates_near_polygon(polygon, 0.0))
            .collect::<std::collections::BTreeSet<_>>();
        candidate_apertures += paste_candidates.len();
        if paste_candidates.is_empty() {
            continue;
        }
        let paste_candidates = paste_candidates
            .into_iter()
            .map(|index| paste_polygons[index].clone())
            .collect::<Vec<_>>();
        let paste_candidate_sketch =
            polygons_to_sketch(paste_candidates, Some(metadata(paste_name)));

        let overlap = paste_candidate_sketch.intersection(&via_opening);
        let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "paste-via-exposure-readiness",
            Severity::Warning,
            vec![paste_name.to_string(), via.layer.clone()],
            None,
            shapes,
            vec![via.location],
            Some(
                "paste aperture overlaps a parsed via opening; confirm via fill, cap, tent, or stencil keepout to avoid solder wicking"
                    .to_string(),
            ),
        ));
    }

    log::trace!(
        "paste-via exposure readiness: paste={} vias={} paste_apertures={} paste_buckets={} candidate_apertures={} selected_layers={}",
        paste_name,
        vias.len(),
        paste_polygons.len(),
        paste_index.bucket_count(),
        candidate_apertures,
        selected_layers.len()
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

fn matching_plated_drill<'a>(
    board: &'a BoardModel,
    feature: &CopperFeature,
) -> Option<&'a DrillFeature> {
    board.drills.iter().find(|drill| {
        drill.plated
            && drill.net == feature.net
            && point_distance(drill.location, feature.location) <= drill.diameter.max(0.05)
    })
}

fn polygon_center(polygon: &Polygon<f64>) -> Option<[f64; 2]> {
    let bounds = polygon.bounding_rect()?;
    Some([
        (bounds.min().x + bounds.max().x) / 2.0,
        (bounds.min().y + bounds.max().y) / 2.0,
    ])
}

fn point_distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    (dx * dx + dy * dy).sqrt()
}

fn metadata(layer_name: &str) -> LayerMetadata {
    LayerMetadata {
        name: layer_name.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        paste_aperture_aspect_ratio_readiness, paste_via_exposure_readiness,
        stencil_area_ratio_readiness, thermal_pad_paste_windowpane_readiness,
        tombstone_paste_imbalance_readiness,
    };
    use crate::geometry::{circle_polygon, polygons_to_sketch};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
    use crate::{LayerMetadata, PcbSketch};
    use geo::{Coord, LineString, Polygon};

    #[test]
    fn thermal_pad_paste_windowpane_readiness_reports_single_large_aperture() {
        let copper = sketch("top", vec![square(0.0, 0.0, 4.0, 4.0)]);
        let paste = sketch("paste", vec![square(0.2, 0.2, 3.8, 3.8)]);

        let violations = thermal_pad_paste_windowpane_readiness(
            "paste", &paste, "top", &copper, 4.0, 0.65, 1.0e-9,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "thermal-pad-paste-windowpane-readiness"
        );
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("windowpane"))
        );
    }

    #[test]
    fn thermal_pad_paste_windowpane_readiness_accepts_split_or_small_apertures() {
        let copper = sketch("top", vec![square(0.0, 0.0, 4.0, 4.0)]);
        let split_paste = sketch(
            "paste",
            vec![
                square(0.2, 0.2, 1.6, 1.6),
                square(2.4, 0.2, 3.8, 1.6),
                square(0.2, 2.4, 1.6, 3.8),
                square(2.4, 2.4, 3.8, 3.8),
            ],
        );
        let reduced_paste = sketch("paste", vec![square(0.2, 0.2, 2.0, 2.0)]);

        assert!(
            thermal_pad_paste_windowpane_readiness(
                "paste",
                &split_paste,
                "top",
                &copper,
                4.0,
                0.65,
                1.0e-9
            )
            .is_empty()
        );
        assert!(
            thermal_pad_paste_windowpane_readiness(
                "paste",
                &reduced_paste,
                "top",
                &copper,
                4.0,
                0.65,
                1.0e-9
            )
            .is_empty()
        );
    }

    #[test]
    fn thermal_pad_paste_windowpane_readiness_culls_sparse_paste_fields() {
        let copper = sketch("top", vec![square(0.0, 0.0, 4.0, 4.0)]);
        let paste = sketch(
            "paste",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 5.0;
                    square(x, 0.0, x + 1.0, 1.0)
                })
                .chain([square(0.2, 0.2, 3.8, 3.8)])
                .collect(),
        );

        let started = std::time::Instant::now();
        let violations = thermal_pad_paste_windowpane_readiness(
            "paste", &paste, "top", &copper, 4.0, 0.65, 1.0e-9,
        );

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "thermal-pad paste windowpane should index sparse paste fields"
        );
    }

    #[test]
    fn paste_aperture_aspect_ratio_readiness_reports_long_sliver_apertures() {
        let paste = sketch("paste", vec![square(0.0, 0.0, 5.0, 0.5)]);

        let violations = paste_aperture_aspect_ratio_readiness("paste", &paste, 4.0, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "paste-aperture-aspect-ratio-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("stencil release"))
        );
    }

    #[test]
    fn stencil_area_ratio_readiness_reports_low_area_ratio_apertures() {
        let paste = sketch("paste", vec![square(0.0, 0.0, 0.18, 0.18)]);

        let violations = stencil_area_ratio_readiness("paste", &paste, 0.15, 0.66, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "stencil-area-ratio-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("area ratio"))
        );
    }

    #[test]
    fn stencil_area_ratio_readiness_allows_printable_or_unconfigured_apertures() {
        let paste = sketch(
            "paste",
            vec![square(0.0, 0.0, 0.5, 0.5), square(1.0, 0.0, 1.05, 0.05)],
        );

        assert!(stencil_area_ratio_readiness("paste", &paste, 0.15, 0.66, 0.01).is_empty());
        assert!(stencil_area_ratio_readiness("paste", &paste, 0.0, 0.66, 1.0e-9).is_empty());
    }

    #[test]
    fn paste_aperture_aspect_ratio_readiness_allows_compact_apertures() {
        let paste = sketch("paste", vec![square(0.0, 0.0, 1.5, 0.5)]);

        assert!(paste_aperture_aspect_ratio_readiness("paste", &paste, 4.0, 1.0e-9).is_empty());
    }

    #[test]
    fn tombstone_paste_imbalance_readiness_reports_neighboring_pad_imbalance() {
        let copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 2.4, 1.0)],
        );
        let paste = sketch(
            "paste",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 1.9, 1.0)],
        );

        let violations =
            tombstone_paste_imbalance_readiness("paste", &paste, "top", &copper, 2.0, 0.30, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "tombstone-paste-imbalance-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("tombstoning"))
        );
    }

    #[test]
    fn tombstone_paste_imbalance_readiness_allows_balanced_or_distant_pads() {
        let balanced_copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 2.4, 1.0)],
        );
        let balanced_paste = sketch(
            "paste",
            vec![square(0.1, 0.0, 0.9, 1.0), square(1.5, 0.0, 2.3, 1.0)],
        );
        let distant_copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(5.0, 0.0, 6.0, 1.0)],
        );
        let distant_paste = sketch(
            "paste",
            vec![square(0.0, 0.0, 1.0, 1.0), square(5.0, 0.0, 5.5, 1.0)],
        );

        assert!(
            tombstone_paste_imbalance_readiness(
                "paste",
                &balanced_paste,
                "top",
                &balanced_copper,
                2.0,
                0.30,
                1.0e-9
            )
            .is_empty()
        );
        assert!(
            tombstone_paste_imbalance_readiness(
                "paste",
                &distant_paste,
                "top",
                &distant_copper,
                2.0,
                0.30,
                1.0e-9
            )
            .is_empty()
        );
    }

    #[test]
    fn tombstone_paste_imbalance_readiness_culls_sparse_pad_fields() {
        let mut copper_polygons = (0..2_000)
            .map(|index| {
                let x = 100.0 + index as f64 * 5.0;
                square(x, 0.0, x + 1.0, 1.0)
            })
            .collect::<Vec<_>>();
        copper_polygons.push(square(0.0, 0.0, 1.0, 1.0));
        copper_polygons.push(square(1.4, 0.0, 2.4, 1.0));
        let copper = sketch("top", copper_polygons);
        let paste = sketch(
            "paste",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 1.9, 1.0)],
        );

        let started = std::time::Instant::now();
        let violations =
            tombstone_paste_imbalance_readiness("paste", &paste, "top", &copper, 2.0, 0.30, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "tombstone paste imbalance should index sparse pad fields before pair review"
        );
    }

    #[test]
    fn tombstone_paste_imbalance_readiness_culls_sparse_paste_fields() {
        let copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 2.4, 1.0)],
        );
        let paste = sketch(
            "paste",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 5.0;
                    square(x, 0.0, x + 1.0, 1.0)
                })
                .chain([square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 1.9, 1.0)])
                .collect(),
        );

        let started = std::time::Instant::now();
        let violations =
            tombstone_paste_imbalance_readiness("paste", &paste, "top", &copper, 2.0, 0.30, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "tombstone paste imbalance should index sparse paste fields before paste-ratio review"
        );
    }

    #[test]
    fn paste_via_exposure_readiness_reports_paste_over_via_drill() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("GND", CopperKind::Via, [0.0, 0.0], 0.16)],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.20,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let paste = sketch("paste", vec![square(-0.2, -0.2, 0.2, 0.2)]);

        let violations = paste_via_exposure_readiness("F.Paste", &paste, &board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "paste-via-exposure-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("solder wicking"))
        );
    }

    #[test]
    fn paste_via_exposure_readiness_allows_distant_or_unselected_vias() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc_on_layer(
                "GND",
                CopperKind::Via,
                "B.Cu",
                [0.0, 0.0],
                0.16,
            )],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.20,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let distant_paste = sketch("paste", vec![square(1.0, 1.0, 1.4, 1.4)]);
        let overlapping_paste = sketch("paste", vec![square(-0.2, -0.2, 0.2, 0.2)]);

        assert!(
            paste_via_exposure_readiness("B.Paste", &distant_paste, &board, &[], 1.0e-9).is_empty()
        );
        assert!(
            paste_via_exposure_readiness(
                "F.Paste",
                &overlapping_paste,
                &board,
                &["F.Cu".to_string()],
                1.0e-9
            )
            .is_empty()
        );
    }

    #[test]
    fn paste_via_exposure_readiness_culls_sparse_paste_fields() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("GND", CopperKind::Via, [0.0, 0.0], 0.16)],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.20,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let paste = sketch(
            "paste",
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 5.0;
                    square(x, 0.0, x + 1.0, 1.0)
                })
                .chain([square(-0.2, -0.2, 0.2, 0.2)])
                .collect(),
        );

        let started = std::time::Instant::now();
        let violations = paste_via_exposure_readiness("F.Paste", &paste, &board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "paste-via exposure should index sparse paste fields before via-opening review"
        );
    }

    fn sketch(name: &str, polygons: Vec<Polygon<f64>>) -> PcbSketch {
        polygons_to_sketch(
            polygons,
            Some(LayerMetadata {
                name: name.to_string(),
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
}
