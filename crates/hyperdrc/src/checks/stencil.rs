//! Stencil and solder-paste readiness checks.
//!
//! These checks live apart from generic layer and board checks because paste
//! review often combines flattened Gerber aperture geometry with richer KiCad
//! via or drill context.

use crate::geometry::{
    Polygon, multipolygon_area_scalar, multipolygon_to_shapes_scalar, polygon_area_scalar,
    polygon_bounds_scalar, polygon_to_profile, polygons_to_profile,
};
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbRegion, PcbRegionExt, Scalar};

use super::spatial::{LayerPolygonSpatialIndex, PointSpatialIndex};
use super::{intersection_for_check, union_for_check};

/// Warn when a large copper island is pasted as one broad aperture.
///
/// IPC-7525B frames stencil aperture design around paste release and volume
/// control; for bottom-termination thermal pads that usually means reduced or
/// split apertures rather than full pad coverage. Thermal pad solder geometry
/// and vias also affect QFN voiding risk. A layer-polygon broad phase selects
/// paste candidates before exact copper/paste intersection, so sparse exports
/// do not force every
/// thermal pad candidate to scan every aperture.
pub fn thermal_pad_paste_windowpane_readiness(
    paste_name: &str,
    paste: &PcbRegion,
    copper_name: &str,
    copper: &PcbRegion,
    min_copper_area: &Scalar,
    max_single_aperture_ratio: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let paste_polygons = paste.to_multipolygon().0;
    let paste_index = LayerPolygonSpatialIndex::new(&paste_polygons, 0.0);
    let mut violations = Vec::new();
    let mut candidate_apertures = 0usize;

    for (island_index, copper_polygon) in copper.to_multipolygon().0.into_iter().enumerate() {
        let Some(copper_area) = polygon_area_scalar(&copper_polygon) else {
            continue;
        };
        if crate::scalar::lt(&copper_area, min_copper_area) {
            continue;
        }

        let paste_candidates = paste_index.candidates_near_polygon(&copper_polygon, 0.0);
        candidate_apertures += paste_candidates.len();
        let island = polygon_to_profile(copper_polygon, Some(metadata(copper_name)));
        let mut intersecting_apertures = 0usize;
        let mut paste_areas = Vec::new();
        for paste_index in paste_candidates {
            let paste_island = polygon_to_profile(
                paste_polygons[paste_index].clone(),
                Some(metadata(paste_name)),
            );
            let overlap = match intersection_for_check(
                &island,
                &paste_island,
                "thermal-pad-paste-windowpane-readiness",
                vec![paste_name.to_string(), copper_name.to_string()],
            ) {
                Ok(overlap) => overlap,
                Err(uncertainty) => return vec![*uncertainty],
            }
            .to_multipolygon();
            let Some(overlap_area) = multipolygon_area_scalar(&overlap) else {
                continue;
            };
            if crate::scalar::le(&overlap_area, min_area) {
                continue;
            }
            intersecting_apertures += 1;
            paste_areas.push(overlap_area);
        }

        let paste_area = Scalar::sum_owned(paste_areas);
        let Ok(ratio) = paste_area / copper_area else {
            continue;
        };
        if intersecting_apertures >= 2 || crate::scalar::le(&ratio, max_single_aperture_ratio) {
            continue;
        }

        violations.push(Violation::new(
            "thermal-pad-paste-windowpane-readiness",
            Severity::Warning,
            vec![paste_name.to_string(), copper_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes_scalar(&island.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "large copper island has one paste aperture with ratio {ratio:#.3}; review windowpane paste reduction for thermal pad solder voiding"
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
/// wall area and uses it as a paste-transfer metric. HyperDRC estimates wall
/// area from each aperture's
/// bounding rectangle until exact stencil thickness and aperture side-wall
/// models become available.
pub fn stencil_area_ratio_readiness(
    paste_name: &str,
    paste: &PcbRegion,
    stencil_thickness: &Scalar,
    min_area_ratio: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    if crate::scalar::le(stencil_thickness, &Scalar::zero()) {
        return Vec::new();
    }

    let mut violations = Vec::new();
    for (island_index, polygon) in paste.to_multipolygon().0.into_iter().enumerate() {
        let Some(aperture_area) = polygon_area_scalar(&polygon) else {
            continue;
        };
        if crate::scalar::le(&aperture_area, min_area) {
            continue;
        }
        let Some(bounds) = polygon_bounds_scalar(&polygon) else {
            continue;
        };
        let width = &bounds[2] - &bounds[0];
        let height = &bounds[3] - &bounds[1];
        if crate::scalar::le(&width, &Scalar::zero()) || crate::scalar::le(&height, &Scalar::zero())
        {
            continue;
        }

        let wall_area = crate::scalar::scalar("2") * (width + height) * stencil_thickness;
        if crate::scalar::le(&wall_area, &Scalar::zero()) {
            continue;
        }
        let Ok(area_ratio) = aperture_area / wall_area else {
            continue;
        };
        if crate::scalar::ge(&area_ratio, min_area_ratio) {
            continue;
        }

        let aperture = polygon_to_profile(polygon, Some(metadata(paste_name)));
        violations.push(Violation::new(
            "stencil-area-ratio-readiness",
            Severity::Warning,
            vec![paste_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes_scalar(&aperture.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "stencil aperture area ratio {area_ratio:#.3} is below minimum {min_area_ratio:#.3}; review stencil thickness, aperture size, or paste release process"
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
    paste: &PcbRegion,
    max_aspect_ratio: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for (island_index, polygon) in paste.to_multipolygon().0.into_iter().enumerate() {
        if polygon_area_scalar(&polygon).is_none_or(|area| crate::scalar::le(&area, min_area)) {
            continue;
        }
        let Some(bounds) = polygon_bounds_scalar(&polygon) else {
            continue;
        };
        let width = &bounds[2] - &bounds[0];
        let height = &bounds[3] - &bounds[1];
        let (min_dimension, max_dimension) = if crate::scalar::le(&width, &height) {
            (width, height)
        } else {
            (height, width)
        };
        if crate::scalar::le(&min_dimension, &Scalar::zero()) {
            continue;
        }
        let Ok(aspect_ratio) = max_dimension / min_dimension else {
            continue;
        };
        if crate::scalar::le(&aspect_ratio, max_aspect_ratio) {
            continue;
        }

        let aperture = polygon_to_profile(polygon, Some(metadata(paste_name)));
        violations.push(Violation::new(
            "paste-aperture-aspect-ratio-readiness",
            Severity::Warning,
            vec![paste_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes_scalar(&aperture.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "paste aperture aspect ratio {aspect_ratio:#.3} exceeds {max_aspect_ratio:#.3}; review stencil release and slumping risk"
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
/// area-ratio and paste-ratio checks. Paste coverage uses the same broad-phase
/// idea over aperture polygons before exact copper/paste intersection, so large
/// sparse stencil exports do not force every pad to intersect every aperture.
pub fn tombstone_paste_imbalance_readiness(
    paste_name: &str,
    paste: &PcbRegion,
    copper_name: &str,
    copper: &PcbRegion,
    max_pair_gap: &Scalar,
    max_ratio_delta: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let copper_polygons = copper.to_multipolygon().0;
    let paste_polygons = paste.to_multipolygon().0;
    let paste_index = LayerPolygonSpatialIndex::new(&paste_polygons, 0.0);
    let mut islands = Vec::new();
    let mut paste_candidate_polygons = 0usize;
    for (index, polygon) in copper_polygons.into_iter().enumerate() {
        let Some(area) = polygon_area_scalar(&polygon) else {
            continue;
        };
        if crate::scalar::le(&area, min_area) {
            continue;
        }
        let Some(center) = polygon_center_scalar(&polygon) else {
            continue;
        };
        let paste_candidates = paste_index.candidates_near_polygon(&polygon, 0.0);
        paste_candidate_polygons += paste_candidates.len();
        let island = polygon_to_profile(polygon, Some(metadata(copper_name)));
        let mut paste_areas = Vec::new();
        for paste_index in paste_candidates {
            let paste_island = polygon_to_profile(
                paste_polygons[paste_index].clone(),
                Some(metadata(paste_name)),
            );
            let overlap = match intersection_for_check(
                &island,
                &paste_island,
                "tombstone-paste-imbalance-readiness",
                vec![paste_name.to_string(), copper_name.to_string()],
            ) {
                Ok(overlap) => overlap,
                Err(uncertainty) => return vec![*uncertainty],
            };
            if let Some(area) = multipolygon_area_scalar(&overlap.to_multipolygon()) {
                paste_areas.push(area);
            }
        }
        let paste_area = Scalar::sum_owned(paste_areas);
        let Ok(paste_ratio) = paste_area / &area else {
            continue;
        };
        islands.push((index, island, center, area, paste_ratio));
    }

    let broad_phase_gap = scalar_broad_phase_radius(max_pair_gap);
    let center_index = PointSpatialIndex::new(
        islands
            .iter()
            .map(|(_, _, center, _, _)| scalar_point_f64_compatibility(center)),
        broad_phase_gap,
    );
    let mut candidate_pairs = 0_usize;
    let mut violations = Vec::new();
    for left_index in 0..islands.len() {
        let (left_original_index, left_island, left_center, left_area, left_ratio) =
            &islands[left_index];
        for right_index in center_index
            .candidate_centers_near(scalar_point_f64_compatibility(left_center), broad_phase_gap)
        {
            if right_index <= left_index {
                continue;
            }
            candidate_pairs += 1;
            let (right_original_index, right_island, right_center, right_area, right_ratio) =
                &islands[right_index];
            if exact_point_distance_scalar(left_center, right_center)
                .is_none_or(|distance| crate::scalar::gt(&distance, max_pair_gap))
            {
                continue;
            }
            let (larger_area, smaller_area) = if crate::scalar::ge(left_area, right_area) {
                (left_area, right_area)
            } else {
                (right_area, left_area)
            };
            let Ok(area_ratio) = larger_area.clone() / smaller_area else {
                continue;
            };
            if crate::scalar::gt(&area_ratio, &crate::scalar::scalar("1.5")) {
                continue;
            }
            let delta = (left_ratio - right_ratio).abs();
            if crate::scalar::le(&delta, max_ratio_delta) {
                continue;
            }

            let combined = match union_for_check(
                left_island,
                right_island,
                "tombstone-paste-imbalance-readiness",
                vec![paste_name.to_string(), copper_name.to_string()],
            ) {
                Ok(combined) => combined,
                Err(uncertainty) => return vec![*uncertainty],
            };
            violations.push(Violation::new(
                "tombstone-paste-imbalance-readiness",
                Severity::Warning,
                vec![paste_name.to_string(), copper_name.to_string()],
                Some(*left_original_index.min(right_original_index)),
                multipolygon_to_shapes_scalar(&combined.to_multipolygon(), min_area),
                Vec::new(),
                Some(format!(
                    "neighboring small copper islands have paste ratio imbalance {delta:#.3}; review tombstoning risk on two-terminal components"
                )),
            ));
        }
    }
    log::trace!(
        "tombstone paste imbalance readiness: paste={} copper={} islands={} paste_buckets={} paste_candidate_polygons={} pair_buckets={} candidate_pairs={} max_pair_gap={max_pair_gap:#.6} violations={}",
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
    paste: &PcbRegion,
    board: &BoardModel,
    selected_layers: &[String],
    min_area: &Scalar,
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
                let radius = crate::scalar::half(&drill.diameter);
                PcbRegion::new(
                    crate::translated_circle(
                        radius,
                        48,
                        drill.location[0].clone(),
                        drill.location[1].clone(),
                    ),
                    Some(LayerMetadata {
                        name: "via drill opening".to_string(),
                    }),
                )
            })
            .unwrap_or_else(|| via.region.clone());
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
        let paste_candidate_region =
            polygons_to_profile(paste_candidates, Some(metadata(paste_name)));

        let overlap = match intersection_for_check(
            &paste_candidate_region,
            &via_opening,
            "paste-via-exposure-readiness",
            vec![paste_name.to_string(), via.layer.clone()],
        ) {
            Ok(overlap) => overlap,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "paste-via-exposure-readiness",
            Severity::Warning,
            vec![paste_name.to_string(), via.layer.clone()],
            None,
            shapes,
            vec![via.location_f64_compatibility_required()],
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
        let matching_radius = if crate::scalar::ge(&drill.diameter, &crate::scalar::scalar("0.05"))
        {
            drill.diameter.clone()
        } else {
            crate::scalar::scalar("0.05")
        };
        drill.plated
            && drill.net == feature.net
            && exact_point_distance_scalar(&drill.location, &feature.location)
                .is_some_and(|distance| crate::scalar::le(&distance, &matching_radius))
    })
}

fn polygon_center_scalar(polygon: &Polygon<f64>) -> Option<[Scalar; 2]> {
    let bounds = polygon_bounds_scalar(polygon)?;
    Some([
        crate::scalar::half(&(&bounds[0] + &bounds[2])),
        crate::scalar::half(&(&bounds[1] + &bounds[3])),
    ])
}

fn scalar_point_f64_compatibility(point: &[Scalar; 2]) -> [f64; 2] {
    [
        point[0]
            .to_f64_lossy()
            .expect("stencil broad-phase x coordinate must be finite"),
        point[1]
            .to_f64_lossy()
            .expect("stencil broad-phase y coordinate must be finite"),
    ]
}

fn exact_point_distance_scalar(left: &[Scalar; 2], right: &[Scalar; 2]) -> Option<Scalar> {
    let dx = &left[0] - &right[0];
    let dy = &left[1] - &right[1];
    (&dx * &dx + &dy * &dy).sqrt().ok()
}

fn scalar_broad_phase_radius(value: &Scalar) -> f64 {
    let projected = value
        .to_f64_lossy()
        .expect("stencil broad-phase radius must be finite");
    if projected > 0.0 {
        projected.next_up()
    } else {
        0.0
    }
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
    use crate::geometry::{Coord, LineString, Polygon};
    use crate::geometry::{circle_polygon, polygons_to_profile};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
    use crate::scalar::scalar;
    use crate::{LayerMetadata, PcbRegion};

    #[test]
    fn thermal_pad_paste_windowpane_readiness_reports_single_large_aperture() {
        let copper = region("top", vec![square(0.0, 0.0, 4.0, 4.0)]);
        let paste = region("paste", vec![square(0.2, 0.2, 3.8, 3.8)]);

        let violations = thermal_pad_paste_windowpane_readiness(
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("4.0"),
            &crate::scalar::scalar("0.65"),
            &crate::scalar::scalar("1.0e-9"),
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
        let copper = region("top", vec![square(0.0, 0.0, 4.0, 4.0)]);
        let split_paste = region(
            "paste",
            vec![
                square(0.2, 0.2, 1.6, 1.6),
                square(2.4, 0.2, 3.8, 1.6),
                square(0.2, 2.4, 1.6, 3.8),
                square(2.4, 2.4, 3.8, 3.8),
            ],
        );
        let reduced_paste = region("paste", vec![square(0.2, 0.2, 2.0, 2.0)]);

        assert!(
            thermal_pad_paste_windowpane_readiness(
                "paste",
                &split_paste,
                "top",
                &copper,
                &crate::scalar::scalar("4.0"),
                &crate::scalar::scalar("0.65"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
        assert!(
            thermal_pad_paste_windowpane_readiness(
                "paste",
                &reduced_paste,
                "top",
                &copper,
                &crate::scalar::scalar("4.0"),
                &crate::scalar::scalar("0.65"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn thermal_pad_paste_windowpane_readiness_culls_sparse_paste_fields() {
        let copper = region("top", vec![square(0.0, 0.0, 4.0, 4.0)]);
        let paste = region(
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
            "paste",
            &paste,
            "top",
            &copper,
            &crate::scalar::scalar("4.0"),
            &crate::scalar::scalar("0.65"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "thermal-pad paste windowpane should index sparse paste fields"
        );
    }

    #[test]
    fn paste_aperture_aspect_ratio_readiness_reports_long_sliver_apertures() {
        let paste = region("paste", vec![square(0.0, 0.0, 5.0, 0.5)]);

        let violations = paste_aperture_aspect_ratio_readiness(
            "paste",
            &paste,
            &crate::scalar::scalar("4.0"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
        let paste = region("paste", vec![square(0.0, 0.0, 0.18, 0.18)]);

        let violations = stencil_area_ratio_readiness(
            "paste",
            &paste,
            &crate::scalar::scalar("0.15"),
            &crate::scalar::scalar("0.66"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
        let paste = region(
            "paste",
            vec![square(0.0, 0.0, 0.5, 0.5), square(1.0, 0.0, 1.05, 0.05)],
        );

        assert!(
            stencil_area_ratio_readiness(
                "paste",
                &paste,
                &crate::scalar::scalar("0.15"),
                &crate::scalar::scalar("0.66"),
                &crate::scalar::scalar("0.01"),
            )
            .is_empty()
        );
        assert!(
            stencil_area_ratio_readiness(
                "paste",
                &paste,
                &crate::scalar::scalar("0"),
                &crate::scalar::scalar("0.66"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn paste_aperture_aspect_ratio_readiness_allows_compact_apertures() {
        let paste = region("paste", vec![square(0.0, 0.0, 1.5, 0.5)]);

        assert!(
            paste_aperture_aspect_ratio_readiness(
                "paste",
                &paste,
                &crate::scalar::scalar("4.0"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn tombstone_paste_imbalance_readiness_reports_neighboring_pad_imbalance() {
        let copper = region(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 2.4, 1.0)],
        );
        let paste = region(
            "paste",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 1.9, 1.0)],
        );

        let violations = tombstone_paste_imbalance_readiness(
            "paste",
            &paste,
            "top",
            &copper,
            &scalar("2.0"),
            &scalar("0.30"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
        let balanced_copper = region(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 2.4, 1.0)],
        );
        let balanced_paste = region(
            "paste",
            vec![square(0.1, 0.0, 0.9, 1.0), square(1.5, 0.0, 2.3, 1.0)],
        );
        let distant_copper = region(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(5.0, 0.0, 6.0, 1.0)],
        );
        let distant_paste = region(
            "paste",
            vec![square(0.0, 0.0, 1.0, 1.0), square(5.0, 0.0, 5.5, 1.0)],
        );

        assert!(
            tombstone_paste_imbalance_readiness(
                "paste",
                &balanced_paste,
                "top",
                &balanced_copper,
                &scalar("2.0"),
                &scalar("0.30"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
        assert!(
            tombstone_paste_imbalance_readiness(
                "paste",
                &distant_paste,
                "top",
                &distant_copper,
                &scalar("2.0"),
                &scalar("0.30"),
                &crate::scalar::scalar("1.0e-9")
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
        let copper = region("top", copper_polygons);
        let paste = region(
            "paste",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 1.9, 1.0)],
        );

        let started = std::time::Instant::now();
        let violations = tombstone_paste_imbalance_readiness(
            "paste",
            &paste,
            "top",
            &copper,
            &scalar("2.0"),
            &scalar("0.30"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "tombstone paste imbalance should index sparse pad fields before pair review"
        );
    }

    #[test]
    fn tombstone_paste_imbalance_readiness_culls_sparse_paste_fields() {
        let copper = region(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.4, 0.0, 2.4, 1.0)],
        );
        let paste = region(
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
        let violations = tombstone_paste_imbalance_readiness(
            "paste",
            &paste,
            "top",
            &copper,
            &scalar("2.0"),
            &scalar("0.30"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
                location: [
                    crate::geometry::exact_real(0.0),
                    crate::geometry::exact_real(0.0),
                ],
                diameter: crate::scalar::scalar("0.20"),
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let paste = region("paste", vec![square(-0.2, -0.2, 0.2, 0.2)]);

        let violations = paste_via_exposure_readiness(
            "F.Paste",
            &paste,
            &board,
            &[],
            &crate::scalar::scalar("1.0e-9"),
        );

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
                location: [
                    crate::geometry::exact_real(0.0),
                    crate::geometry::exact_real(0.0),
                ],
                diameter: crate::scalar::scalar("0.20"),
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let distant_paste = region("paste", vec![square(1.0, 1.0, 1.4, 1.4)]);
        let overlapping_paste = region("paste", vec![square(-0.2, -0.2, 0.2, 0.2)]);

        assert!(
            paste_via_exposure_readiness(
                "B.Paste",
                &distant_paste,
                &board,
                &[],
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
        assert!(
            paste_via_exposure_readiness(
                "F.Paste",
                &overlapping_paste,
                &board,
                &["F.Cu".to_string()],
                &crate::scalar::scalar("1.0e-9")
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
                location: [
                    crate::geometry::exact_real(0.0),
                    crate::geometry::exact_real(0.0),
                ],
                diameter: crate::scalar::scalar("0.20"),
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let paste = region(
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
        let violations = paste_via_exposure_readiness(
            "F.Paste",
            &paste,
            &board,
            &[],
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "paste-via exposure should index sparse paste fields before via-opening review"
        );
    }

    fn region(name: &str, polygons: Vec<Polygon<f64>>) -> PcbRegion {
        polygons_to_profile(
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
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            region: polygons_to_profile(
                vec![circle_polygon(location, radius, 32)],
                Some(LayerMetadata {
                    name: "feature".to_string(),
                }),
            ),
        }
    }
}
