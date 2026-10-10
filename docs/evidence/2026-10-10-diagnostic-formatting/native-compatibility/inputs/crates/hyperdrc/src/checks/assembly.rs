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

use crate::checks::outline::{axis_aligned_outline_rect, feature_bounds_inside_rect_margin};

mod mouse_bite;
mod testpoint;
use crate::checks::{intersection_for_check, offset_for_check};
use crate::geometry::multipolygon_to_shapes_scalar;
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbRegion, PcbRegionExt, Scalar};
pub use mouse_bite::mouse_bite_readiness;
pub use testpoint::{
    testpoint_accessibility_readiness, testpoint_copper_clearance_readiness,
    testpoint_coverage_readiness,
};

/// Review component-pad proxies against the assembly edge-clearance band.
///
/// IPC-7351B treats component placement courtyard and fabrication/assembly
/// tolerances as process constraints. Until parsed package courtyards are
/// available, this check uses non-fiducial KiCad pads as conservative body
/// proxies, then applies a rectangular broad phase before exact outline-distance
/// review on simple board rectangles.
pub fn component_edge_clearance_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "component-edge-clearance-readiness",
        &[&board.source],
        || {
            let Some(outline) = &board.board_outline else {
                return Ok(Vec::new());
            };

            let outline_rect = axis_aligned_outline_rect(outline);
            let broad_phase_clearance = scalar_broad_phase_radius(clearance);
            let candidate_pads = selected_copper_features(board, selected_layers)
                .into_iter()
                .filter(|feature| feature.kind == CopperKind::Pad)
                .filter(|feature| !likely_fiducial(feature))
                .filter(|feature| !feature.net.as_deref().is_some_and(looks_edge_intent_net))
                .collect::<Vec<_>>();
            let mut rect_rejections = 0_usize;
            let mut exact_checks = 0_usize;
            let mut violations = Vec::new();
            for feature in &candidate_pads {
                if outline_rect.as_ref().is_some_and(|rect| {
                    feature_bounds_inside_rect_margin(feature, rect, broad_phase_clearance)
                }) {
                    rect_rejections += 1;
                    continue;
                }
                exact_checks += 1;
                let Some(edge_gap) = crate::checks::boundary_distance_for_check(
                    &feature.region,
                    outline,
                    "component-edge-clearance-readiness",
                    &[&feature.layer],
                    &mut violations,
                ) else {
                    continue;
                };
                if crate::checks::metric_cmp_for_check(
                    &edge_gap,
                    clearance,
                    "component-edge-clearance-readiness",
                    &[&feature.layer],
                    &mut violations,
                )
                .is_some_and(|order| order.is_lt())
                {
                    violations.push(Violation::new(
                    "component-edge-clearance-readiness", Severity::Warning,
                    vec![feature.layer.clone()], None, Vec::new(),
                    vec![feature.try_location_f64()?],
                    Some(format!(
                        "component pad is {edge_gap:#.6} from board edge, below assembly edge clearance {clearance:#.6}; review pick-and-place, depanelization, clamp, and rework access",
                        edge_gap = edge_gap.diagnostic_decimal(-256),
                        clearance = clearance.diagnostic_decimal(-256),
                    )),
                ));
                }
            }

            log::trace!(
                "component-edge clearance readiness: source={} pads={} rect_rejections={} exact_checks={} clearance={clearance:#.6} violations={}",
                board.source,
                candidate_pads.len(),
                rect_rejections,
                exact_checks,
                violations.len(),
                clearance = clearance.diagnostic_decimal(-256),
            );

            Ok(violations)
        },
    )
}

/// Review component-pad proxies against non-plated mechanical-hole keepouts.
///
/// IPC-7351B treats component placement envelopes, courtyard spacing, and
/// board-edge/process keepouts as assembly constraints. Because the current
/// model does not carry full package bodies, this check uses parsed pads as
/// conservative component proxies and applies a spatial broad phase before
/// exact circular keepout intersection.
pub fn component_hole_clearance_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    selected_layers: &[String],
    clearance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "component-hole-clearance-readiness",
        &[&board.source],
        || {
            let mut mechanical_drills = board
                .drills
                .iter()
                .chain(extra_drills.iter())
                .filter(|drill| !drill.plated)
                .collect::<Vec<_>>();
            mechanical_drills.sort_by(|left, right| {
                crate::scalar::compare(&left.location[0], &right.location[0])
                    .expect("exact drill x coordinates must be comparable")
                    .then_with(|| {
                        crate::scalar::compare(&left.location[1], &right.location[1])
                            .expect("exact drill y coordinates must be comparable")
                    })
                    .then_with(|| {
                        crate::scalar::compare(&left.diameter, &right.diameter)
                            .expect("exact drill diameters must be comparable")
                    })
            });

            let pads = selected_copper_features(board, selected_layers)
                .into_iter()
                .filter(|feature| feature.kind == CopperKind::Pad)
                .filter(|feature| !likely_fiducial(feature))
                .collect::<Vec<_>>();
            let broad_phase_clearance = scalar_broad_phase_radius(clearance);
            let pad_index = FeatureGridIndex::new(&pads, broad_phase_clearance);
            let mut violations = Vec::new();
            let mut candidate_count = 0_usize;
            log::trace!(
                "component-hole clearance readiness: source={} mechanical_drills={} pads={} buckets={} clearance={clearance:#.6} min_area={min_area:#.9}",
                board.source,
                mechanical_drills.len(),
                pads.len(),
                pad_index.bucket_count(),
                clearance = clearance.diagnostic_decimal(-256),
                min_area = min_area.diagnostic_decimal(-256),
            );

            for drill in mechanical_drills {
                let drill_radius = crate::scalar::half(&drill.diameter);
                let keepout_radius = drill_radius + clearance;
                let broad_phase_radius = scalar_broad_phase_radius(&keepout_radius);
                let center = drill.try_location_f64()?;
                let broad_candidates = pad_index.near_circle(center, broad_phase_radius);
                candidate_count += broad_candidates.len();
                let candidates = broad_candidates
                    .into_iter()
                    .filter(|&index| {
                        feature_may_touch_circle(pads[index], center, broad_phase_radius)
                    })
                    .collect::<Vec<_>>();
                if candidates.is_empty() {
                    continue;
                }
                let keepout = PcbRegion::new(
                    crate::geometry::circle_region(
                        &[drill.location[0].clone(), drill.location[1].clone()],
                        &(keepout_radius),
                    )?,
                    Some(LayerMetadata {
                        name: "mechanical hole keepout".to_string(),
                    }),
                );

                for pad_index in candidates {
                    let pad = pads[pad_index];
                    let overlap = match intersection_for_check(
                        &keepout,
                        &pad.region,
                        "component-hole-clearance-readiness",
                        vec![pad.layer.clone()],
                    ) {
                        Ok(overlap) => overlap,
                        Err(uncertainty) => return Ok(vec![*uncertainty]),
                    };
                    let shapes =
                        multipolygon_to_shapes_scalar(&overlap.try_to_multipolygon()?, min_area)?;
                    let fallback_hit = shapes.is_empty()
                        && (crate::checks::boundaries_within_for_check(
                            &keepout,
                            &pad.region,
                            &Scalar::zero(),
                            "component-hole-clearance-readiness",
                            &[&pad.layer],
                            &mut violations,
                        ) == Some(true));
                    if shapes.is_empty() && !fallback_hit {
                        continue;
                    }

                    violations.push(Violation::new(
                "component-hole-clearance-readiness",
                Severity::Warning,
                vec![pad.layer.clone()],
                None,
                shapes,
                vec![drill.try_location_f64()?, pad.try_location_f64()?],
                Some(format!(
                    "component pad is within mechanical hole clearance {clearance:#.6}; review screw, standoff, slot, chassis, or connector keepout",
                    clearance = clearance.diagnostic_decimal(-256),
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

            Ok(violations)
        },
    )
}

/// Review spacing between large component-pad proxies on the same side.
///
/// IPC-7351B frames courtyard spacing as an assembly-process contract rather
/// than only a copper DRC rule. Until parsed package courtyards are available,
/// this readiness check uses large KiCad pads as conservative body proxies and
/// runs exact polygon distance only after shared spatial candidate generation.
pub fn component_spacing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: &Scalar,
    minimum_pad_dimension: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("component-spacing-readiness", &[&board.source], || {
        let mut violations = Vec::new();
        let mut pads = Vec::new();
        for feature in selected_copper_features(board, selected_layers)
            .into_iter()
            .filter(|feature| feature.kind == CopperKind::Pad && !likely_fiducial(feature))
        {
            let Some(Some(dimension)) = crate::checks::geometry_result_for_check(
                super::minimum_bounding_dimension(&feature.region),
                "component-spacing-readiness",
                &[&feature.layer],
                &mut violations,
            ) else {
                continue;
            };
            if crate::checks::metric_cmp_for_check(
                &dimension,
                minimum_pad_dimension,
                "component-spacing-readiness",
                &[&feature.layer],
                &mut violations,
            )
            .is_some_and(|order| order.is_ge())
            {
                pads.push(feature);
            }
        }

        let broad_phase_clearance = scalar_broad_phase_radius(clearance);
        let candidate_pairs = same_layer_feature_candidate_pairs(&pads, broad_phase_clearance);
        log::trace!(
            "component spacing readiness: source={} pads={} candidate_pairs={} clearance={clearance:#.6} minimum_pad_dimension={minimum_pad_dimension:#.6}",
            board.source,
            pads.len(),
            candidate_pairs.len(),
            clearance = clearance.diagnostic_decimal(-256),
            minimum_pad_dimension = minimum_pad_dimension.diagnostic_decimal(-256),
        );
        let candidate_pair_count = candidate_pairs.len();
        let mut exact_pair_count = 0usize;
        for (left_index, right_index) in candidate_pairs {
            let left = pads[left_index];
            let right = pads[right_index];
            if !regiones_within_clearance(&left.region, &right.region, broad_phase_clearance) {
                continue;
            }
            exact_pair_count += 1;

            let Some(gap) = crate::checks::boundary_distance_for_check(
                &left.region,
                &right.region,
                "component-spacing-readiness",
                &[&left.layer],
                &mut violations,
            ) else {
                continue;
            };
            if crate::checks::metric_cmp_for_check(
                &gap,
                clearance,
                "component-spacing-readiness",
                &[&left.layer],
                &mut violations,
            )
            .is_none_or(|order| !order.is_lt())
            {
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
                vec![
                    left.try_location_f64()?,
                    right.try_location_f64()?,
                ],
                Some(format!(
                    "large component pad proxies are {gap:#.6} apart, below assembly component spacing {clearance:#.6}; review courtyard/body clearance and rework access",
                    gap = gap.diagnostic_decimal(-256),
                    clearance = clearance.diagnostic_decimal(-256),
                )),
            ));
        }

        log::trace!(
            "component spacing readiness: source={} exact_pairs={} violations={}",
            board.source,
            exact_pair_count,
            violations.len()
        );
        debug_assert!(exact_pair_count <= candidate_pair_count);

        Ok(violations)
    })
}

/// Review rework clearance around likely connector pads.
///
/// IPC-7711/7721 rework guidance treats connector replacement as a
/// tool-access, thermal-control, and neighboring-component risk. This check
/// infers connector pads from net names, uses the same spatial candidate pass as
/// component spacing, and then reports exact polygon gaps below the configured
/// rework clearance.
pub fn connector_rework_clearance_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: &Scalar,
    minimum_pad_dimension: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "connector-rework-clearance-readiness",
        &[&board.source],
        || {
            let pads = selected_copper_features(board, selected_layers)
                .into_iter()
                .filter(|feature| feature.kind == CopperKind::Pad)
                .filter(|feature| !likely_fiducial(feature))
                .collect::<Vec<_>>();
            let mut violations = Vec::new();
            let mut connector_pads = BTreeSet::new();
            for (index, feature) in pads
                .iter()
                .enumerate()
                .filter(|(_, feature)| feature.net.as_deref().is_some_and(looks_connector_net))
            {
                let Some(Some(dimension)) = crate::checks::geometry_result_for_check(
                    super::minimum_bounding_dimension(&feature.region),
                    "connector-rework-clearance-readiness",
                    &[&feature.layer],
                    &mut violations,
                ) else {
                    continue;
                };
                if crate::checks::metric_cmp_for_check(
                    &dimension,
                    minimum_pad_dimension,
                    "connector-rework-clearance-readiness",
                    &[&feature.layer],
                    &mut violations,
                )
                .is_some_and(|order| order.is_ge())
                {
                    connector_pads.insert(index);
                }
            }

            let broad_phase_clearance = scalar_broad_phase_radius(clearance);
            let candidate_pairs = same_layer_feature_candidate_pairs(&pads, broad_phase_clearance);
            log::trace!(
                "connector rework clearance readiness: source={} pads={} connectors={} candidate_pairs={} clearance={clearance:#.6} minimum_pad_dimension={minimum_pad_dimension:#.6}",
                board.source,
                pads.len(),
                connector_pads.len(),
                candidate_pairs.len(),
                clearance = clearance.diagnostic_decimal(-256),
                minimum_pad_dimension = minimum_pad_dimension.diagnostic_decimal(-256),
            );
            let candidate_pair_count = candidate_pairs.len();
            let mut exact_pair_count = 0usize;
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
                exact_pair_count += 1;

                let Some(gap) = crate::checks::boundary_distance_for_check(
                    &connector.region,
                    &neighbor.region,
                    "connector-rework-clearance-readiness",
                    &[&connector.layer],
                    &mut violations,
                ) else {
                    continue;
                };
                if crate::checks::metric_cmp_for_check(
                    &gap,
                    clearance,
                    "connector-rework-clearance-readiness",
                    &[&connector.layer],
                    &mut violations,
                )
                .is_none_or(|order| !order.is_lt())
                {
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
                vec![
                    connector.try_location_f64()?,
                    neighbor.try_location_f64()?,
                ],
                Some(format!(
                    "likely connector pad on net {:?} is {gap:#.6} from neighboring pad, below rework clearance {clearance:#.6}; review soldering iron and connector rework access",
                    connector.net,
                    gap = gap.diagnostic_decimal(-256),
                    clearance = clearance.diagnostic_decimal(-256),
                )),
            ));
            }

            log::trace!(
                "connector rework clearance readiness: source={} exact_pairs={} violations={}",
                board.source,
                exact_pair_count,
                violations.len()
            );
            debug_assert!(exact_pair_count <= candidate_pair_count);

            Ok(violations)
        },
    )
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
    max_pair_gap: &Scalar,
    max_area_ratio: &Scalar,
    max_pad_dimension: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("pad-pair-asymmetry-readiness", &[&board.source], || {
        let mut pads = Vec::new();
        for feature in selected_copper_features(board, selected_layers) {
            if feature.kind != CopperKind::Pad || likely_fiducial(feature) {
                continue;
            }
            let area = feature.region.try_area()?;
            let Some((_, max_dimension)) =
                super::measurements::bounding_dimensions(&feature.region)?
            else {
                continue;
            };
            if crate::scalar::try_compare(&area, &Scalar::zero())?.is_gt()
                && crate::scalar::try_compare(&max_dimension, max_pad_dimension)?.is_le()
            {
                pads.push((feature, area, max_dimension));
            }
        }
        let mut violations = Vec::new();

        let pad_features = pads
            .iter()
            .map(|(feature, _, _)| *feature)
            .collect::<Vec<_>>();
        let broad_phase_gap = scalar_broad_phase_radius(max_pair_gap);
        let candidate_pairs = same_layer_feature_candidate_pairs(&pad_features, broad_phase_gap);
        let candidate_pair_count = candidate_pairs.len();
        let mut exact_pair_count = 0_usize;
        log::trace!(
            "pad-pair asymmetry readiness: source={} pads={} candidate_pairs={} max_pair_gap={max_pair_gap:#.6} max_area_ratio={max_area_ratio:#.3} max_pad_dimension={max_pad_dimension:#.6}",
            board.source,
            pads.len(),
            candidate_pair_count,
            max_pair_gap = max_pair_gap.diagnostic_decimal(-256),
            max_area_ratio = max_area_ratio.diagnostic_decimal(-256),
            max_pad_dimension = max_pad_dimension.diagnostic_decimal(-256),
        );
        for (left_index, right_index) in candidate_pairs {
            let (left, ref left_area, _) = pads[left_index];
            let (right, ref right_area, _) = pads[right_index];
            if left.net.is_some() && left.net == right.net {
                continue;
            }
            if !regiones_within_clearance(&left.region, &right.region, broad_phase_gap) {
                continue;
            }

            let Some(gap) = crate::checks::boundary_distance_for_check(
                &left.region,
                &right.region,
                "pad-pair-asymmetry-readiness",
                &[&left.layer],
                &mut violations,
            ) else {
                continue;
            };
            if crate::checks::metric_cmp_for_check(
                &gap,
                max_pair_gap,
                "pad-pair-asymmetry-readiness",
                &[&left.layer],
                &mut violations,
            )
            .is_none_or(|order| order.is_gt())
            {
                continue;
            }
            exact_pair_count += 1;

            let (larger_area, smaller_area) =
                if crate::scalar::try_compare(left_area, right_area)?.is_ge() {
                    (left_area, right_area)
                } else {
                    (right_area, left_area)
                };
            let area_ratio = crate::scalar::try_divide(larger_area, smaller_area)?;
            if crate::scalar::try_compare(&area_ratio, max_area_ratio)?.is_le() {
                continue;
            }

            violations.push(Violation::new(
            "pad-pair-asymmetry-readiness",
            Severity::Warning,
            vec![left.layer.clone()],
            None,
            Vec::new(),
            vec![
                left.try_location_f64()?,
                right.try_location_f64()?,
            ],
            Some(format!(
                "neighboring small pads have copper area ratio {area_ratio:#.3}, above {max_area_ratio:#.3}; review two-terminal land pattern symmetry and tombstoning risk",
                area_ratio = area_ratio.diagnostic_decimal(-256),
                max_area_ratio = max_area_ratio.diagnostic_decimal(-256),
            )),
        ));
        }

        log::trace!(
            "pad-pair asymmetry readiness: source={} exact_pairs={} violations={}",
            board.source,
            exact_pair_count,
            violations.len()
        );
        debug_assert!(exact_pair_count <= candidate_pair_count);

        Ok(violations)
    })
}

/// Review likely non-plated tooling holes for count and edge clearance.
///
/// Tooling holes are filtered by plated state and finished diameter before
/// outline proximity is checked. The edge review stays exact through
/// boundary-distance geometry. IPC-7351B and IPC-9252B-style assembly/test
/// planning both rely on repeatable fixture registration, so sparse drill-table
/// trace counters make missing or edge-crowded tooling evidence visible before
/// release.
pub fn tooling_hole_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    minimum_diameter: &Scalar,
    maximum_diameter: &Scalar,
    edge_clearance: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("tooling-hole-readiness", &[&board.source], || {
        let mut drills = board.drills.clone();
        drills.extend_from_slice(extra_drills);

        let candidates = drills
            .iter()
            .filter(|drill| !drill.plated)
            .filter(|drill| {
                crate::scalar::ge(&drill.diameter, minimum_diameter)
                    && crate::scalar::le(&drill.diameter, maximum_diameter)
            })
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
                candidates.iter().map(|drill| drill.try_location_f64()).collect::<Result<Vec<_>, _>>()?,
                Some(format!(
                    "found {} likely tooling hole(s); assembly panels usually need at least two non-plated tooling holes between {minimum_diameter:#.6} and {maximum_diameter:#.6}",
                    candidates.len(),
                    minimum_diameter = minimum_diameter.diagnostic_decimal(-256),
                    maximum_diameter = maximum_diameter.diagnostic_decimal(-256),
                )),
            ));
        }

        if let Some(outline) = &board.board_outline {
            for drill in &candidates {
                edge_checks += 1;
                let radius = crate::scalar::half(&drill.diameter);
                let Some(circle) = crate::checks::geometry_result_for_check(
                    crate::geometry::circle_region(&drill.location, &radius),
                    "tooling-hole-readiness",
                    &[&board.source],
                    &mut violations,
                ) else {
                    continue;
                };
                let keepout = PcbRegion::new(
                    circle,
                    Some(LayerMetadata {
                        name: "tooling hole".to_string(),
                    }),
                );
                let Some(edge_gap) = crate::checks::boundary_distance_for_check(
                    &keepout,
                    outline,
                    "tooling-hole-readiness",
                    &[&board.source],
                    &mut violations,
                ) else {
                    continue;
                };
                if crate::checks::metric_cmp_for_check(
                    &edge_gap,
                    edge_clearance,
                    "tooling-hole-readiness",
                    &[&board.source],
                    &mut violations,
                )
                .is_none_or(|order| !order.is_lt())
                {
                    continue;
                }

                violations.push(Violation::new(
                    "tooling-hole-readiness",
                    Severity::Warning,
                    vec!["tooling-holes".to_string()],
                    None,
                    Vec::new(),
                    vec![drill.try_location_f64()?],
                    Some(format!(
                        "likely tooling hole is {edge_gap:#.6} from board edge, below fixture edge clearance {edge_clearance:#.6}",
                        edge_gap = edge_gap.diagnostic_decimal(-256),
                        edge_clearance = edge_clearance.diagnostic_decimal(-256),
                    )),
                ));
            }
        }

        log::trace!(
            "tooling-hole readiness: source={} board_drills={} extra_drills={} candidates={} edge_checks={} minimum_diameter={minimum_diameter:#.6} maximum_diameter={maximum_diameter:#.6} edge_clearance={edge_clearance:#.6} violations={}",
            board.source,
            board.drills.len(),
            extra_drills.len(),
            candidates.len(),
            edge_checks,
            violations.len(),
            minimum_diameter = minimum_diameter.diagnostic_decimal(-256),
            maximum_diameter = maximum_diameter.diagnostic_decimal(-256),
            edge_clearance = edge_clearance.diagnostic_decimal(-256),
        );

        Ok(violations)
    })
}

/// Infer global fiducials and review count plus edge clearance by board side.
///
/// IPC-7351B treats fiducials as assembly registration targets with optical
/// keepout needs. This readiness check infers likely unnetted pad targets and,
/// on rectangular boards, rejects clearly interior candidates
/// before exact outline-distance review.
pub fn fiducial_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("fiducial-readiness", &[&board.source], || {
        let broad_phase_clearance = scalar_broad_phase_radius(edge_clearance);
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
            .filter(|feature| {
                selected_layers.is_empty() || selected_layers.contains(&feature.layer)
            })
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
                    candidates.iter().map(|feature| feature.try_location_f64()).collect::<Result<Vec<_>, _>>()?,
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
                        feature_bounds_inside_rect_margin(candidate, rect, broad_phase_clearance)
                    }) {
                        rect_rejections += 1;
                        continue;
                    }
                    exact_edge_checks += 1;
                    let Some(distance_to_edge) = crate::checks::boundary_distance_for_check(
                        &candidate.region,
                        outline,
                        "fiducial-readiness",
                        &[&candidate.layer],
                        &mut violations,
                    ) else {
                        continue;
                    };
                    if crate::checks::metric_cmp_for_check(
                        &distance_to_edge,
                        edge_clearance,
                        "fiducial-readiness",
                        &[&candidate.layer],
                        &mut violations,
                    )
                    .is_none_or(|order| !order.is_lt())
                    {
                        continue;
                    }
                    violations.push(Violation::new(
                        "fiducial-readiness",
                        Severity::Warning,
                        vec![layer.clone()],
                        None,
                        Vec::new(),
                        vec![candidate.try_location_f64()?],
                        Some(format!(
                            "likely fiducial is {:#.6} from board edge, below clearance {:#.6}",
                            (distance_to_edge).diagnostic_decimal(-256),
                            (edge_clearance).diagnostic_decimal(-256),
                        )),
                    ));
                }
            }
        }

        log::trace!(
            "fiducial readiness: source={} layers={} edge_candidates={} rect_rejections={} exact_edge_checks={} edge_clearance={edge_clearance:#.6} violations={}",
            board.source,
            candidates_by_layer.len(),
            edge_candidates,
            rect_rejections,
            exact_edge_checks,
            violations.len(),
            edge_clearance = edge_clearance.diagnostic_decimal(-256),
        );

        Ok(violations)
    })
}

/// Review likely fiducial targets for same-layer copper keepout intrusions.
///
/// IPC-7351B treats fiducials as placement registration features. This check
/// models the optical clearance annulus as an offset target region, uses a
/// feature-grid broad phase, and then reports exact CSG or boundary-touch
/// intrusions that can reduce camera contrast.
pub fn fiducial_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("fiducial-keepout-readiness", &[&board.source], || {
        let broad_phase_clearance = scalar_broad_phase_radius(clearance);
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
        let blocker_index = FeatureGridIndex::new(&blockers, broad_phase_clearance);
        let mut violations = Vec::new();
        let mut candidate_count = 0_usize;
        let mut exact_pair_count = 0_usize;
        log::trace!(
            "fiducial keepout readiness: source={} fiducials={} blockers={} buckets={} clearance={clearance:#.6} min_area={min_area:#.9}",
            board.source,
            fiducials.len(),
            blockers.len(),
            blocker_index.bucket_count(),
            clearance = clearance.diagnostic_decimal(-256),
            min_area = min_area.diagnostic_decimal(-256),
        );

        for fiducial in fiducials {
            let keepout = match offset_for_check(
                &fiducial.region,
                clearance.clone(),
                "fiducial-keepout-readiness",
                vec![fiducial.layer.clone()],
            ) {
                Ok(keepout) => keepout,
                Err(uncertainty) => return Ok(vec![*uncertainty]),
            };
            for blocker_index in blocker_index.near_circle(
                fiducial.try_location_f64()?,
                feature_query_radius(fiducial, broad_phase_clearance),
            ) {
                candidate_count += 1;
                let blocker = blockers[blocker_index];
                if fiducial.layer != blocker.layer {
                    continue;
                }
                if !regiones_within_clearance(
                    &fiducial.region,
                    &blocker.region,
                    broad_phase_clearance,
                ) {
                    continue;
                }
                exact_pair_count += 1;

                // IPC-7351B treats fiducials as assembly registration features. A
                // clear copper-free annulus around the target improves optical
                // contrast for placement cameras; this models that annulus as an
                // offset target region and reports same-layer copper intrusions.
                let overlap = match intersection_for_check(
                    &keepout,
                    &blocker.region,
                    "fiducial-keepout-readiness",
                    vec![fiducial.layer.clone()],
                ) {
                    Ok(overlap) => overlap,
                    Err(uncertainty) => return Ok(vec![*uncertainty]),
                };
                let shapes =
                    multipolygon_to_shapes_scalar(&overlap.try_to_multipolygon()?, min_area)?;
                let fallback_hit = shapes.is_empty()
                    && (crate::checks::boundaries_within_for_check(
                        &keepout,
                        &blocker.region,
                        &Scalar::zero(),
                        "fiducial-keepout-readiness",
                        &[&blocker.layer],
                        &mut violations,
                    ) == Some(true));
                if shapes.is_empty() && !fallback_hit {
                    continue;
                }

                violations.push(Violation::new(
                "fiducial-keepout-readiness",
                Severity::Warning,
                vec![fiducial.layer.clone()],
                None,
                shapes,
                vec![
                    fiducial.try_location_f64()?,
                    blocker.try_location_f64()?,
                ],
                Some(format!(
                    "likely fiducial has same-layer copper inside optical keepout {clearance:#.6}; review placement-camera contrast, mask opening, and assembly fiducial keepout",
                    clearance = clearance.diagnostic_decimal(-256),
                )),
            ));
            }
        }

        log::trace!(
            "fiducial keepout readiness: source={} candidate_pairs={} exact_pairs={} violations={}",
            board.source,
            candidate_count,
            exact_pair_count,
            violations.len()
        );
        debug_assert!(exact_pair_count <= candidate_count);

        Ok(violations)
    })
}

/// Review likely wave/selective-solder process holes against nearby pads.
///
/// IPC J-STD-001H treats soldering as a controlled process, and IPC-7530
/// profiling guidance reinforces that solder process windows are not captured
/// by copper spacing alone. This check uses plated, solder-process-like drill
/// nets as conservative anchors so reviewers can confirm pallet, solder-thief,
/// and masking intent. A broad phase precedes exact circular keepout intersection.
pub fn selective_wave_solder_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "selective-wave-solder-keepout-readiness",
        &[&board.source],
        || {
            let pads = selected_copper_features(board, selected_layers)
                .into_iter()
                .filter(|feature| feature.kind == CopperKind::Pad)
                .filter(|feature| !likely_fiducial(feature))
                .collect::<Vec<_>>();
            let broad_phase_keepout = scalar_broad_phase_radius(keepout);
            let pad_index = FeatureGridIndex::new(&pads, broad_phase_keepout);
            let solder_drills = board
                .drills
                .iter()
                .filter(|drill| drill.plated)
                .filter(|drill| drill.net.as_deref().is_some_and(looks_solder_process_net))
                .collect::<Vec<_>>();
            let mut violations = Vec::new();
            let mut candidate_count = 0_usize;
            let mut exact_pair_count = 0_usize;
            log::trace!(
                "selective/wave solder keepout readiness: source={} drills={} pads={} buckets={} keepout={keepout:#.6} min_area={min_area:#.9}",
                board.source,
                solder_drills.len(),
                pads.len(),
                pad_index.bucket_count(),
                keepout = keepout.diagnostic_decimal(-256),
                min_area = min_area.diagnostic_decimal(-256),
            );

            for drill in solder_drills {
                let drill_radius = crate::scalar::half(&drill.diameter);
                let keepout_radius = drill_radius + keepout;
                let broad_phase_radius = scalar_broad_phase_radius(&keepout_radius);
                let center = drill.try_location_f64()?;
                let broad_candidates = pad_index.near_circle(center, broad_phase_radius);
                candidate_count += broad_candidates.len();
                let candidates = broad_candidates
                    .into_iter()
                    .filter(|&index| drill.net.is_none() || drill.net != pads[index].net)
                    .filter(|&index| {
                        feature_may_touch_circle(pads[index], center, broad_phase_radius)
                    })
                    .collect::<Vec<_>>();
                if candidates.is_empty() {
                    continue;
                }
                let keepout_region = PcbRegion::new(
                    crate::geometry::circle_region(
                        &[drill.location[0].clone(), drill.location[1].clone()],
                        &(keepout_radius),
                    )?,
                    Some(LayerMetadata {
                        name: "selective/wave solder keepout".to_string(),
                    }),
                );
                for pad_index in candidates {
                    let pad = pads[pad_index];
                    exact_pair_count += 1;
                    let overlap = match intersection_for_check(
                        &keepout_region,
                        &pad.region,
                        "selective-wave-solder-keepout-readiness",
                        vec![pad.layer.clone()],
                    ) {
                        Ok(overlap) => overlap,
                        Err(uncertainty) => return Ok(vec![*uncertainty]),
                    };
                    let shapes =
                        multipolygon_to_shapes_scalar(&overlap.try_to_multipolygon()?, min_area)?;
                    let touching = shapes.is_empty()
                        && (crate::checks::boundaries_within_for_check(
                            &keepout_region,
                            &pad.region,
                            &Scalar::zero(),
                            "selective-wave-solder-keepout-readiness",
                            &[&pad.layer],
                            &mut violations,
                        ) == Some(true));
                    if shapes.is_empty() && !touching {
                        continue;
                    }

                    violations.push(Violation::new(
                "selective-wave-solder-keepout-readiness",
                Severity::Warning,
                vec![pad.layer.clone()],
                None,
                shapes,
                vec![drill.try_location_f64()?, pad.try_location_f64()?],
                Some(format!(
                    "likely through-hole solder feature on net {:?} is within solder-process keepout {keepout:#.6} of neighboring pad {:?}; review selective/wave solder pallet, solder thief, and masking clearance",
                    drill.net,
                    pad.net,
                    keepout = keepout.diagnostic_decimal(-256),
                )),
            ));
                }
            }

            log::trace!(
                "selective/wave solder keepout readiness: source={} candidate_pairs={} exact_pairs={} violations={}",
                board.source,
                candidate_count,
                exact_pair_count,
                violations.len()
            );
            debug_assert!(exact_pair_count <= candidate_count);

            Ok(violations)
        },
    )
}

/// Review press-fit insertion clearance around likely connector holes.
///
/// Press-fit hardware needs insertion-tool and deformation clearance that is not
/// represented by copper clearance alone. IEC 60352-5 press-in guidance
/// motivates the conservative net-name filter and exact keepout intersection.
pub fn press_fit_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: &Scalar,
    min_area: &Scalar,
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
/// likely no-coat features is a useful release-review prompt. A lightweight
/// grid selects neighboring pads before exact keepout/pad CSG
/// intersection.
pub fn conformal_coating_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "conformal-coating-keepout-readiness",
        &[&board.source],
        || {
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
            let broad_phase_keepout = scalar_broad_phase_radius(keepout);
            let pad_index = FeatureGridIndex::new(&pads, broad_phase_keepout);
            let mut violations = Vec::new();
            let mut candidate_count = 0usize;
            let mut exact_pair_count = 0usize;

            for &no_coat in &no_coat_features {
                let keepout_region = match offset_for_check(
                    &no_coat.region,
                    keepout.clone(),
                    "conformal-coating-keepout-readiness",
                    vec![no_coat.layer.clone()],
                ) {
                    Ok(keepout) => keepout,
                    Err(uncertainty) => return Ok(vec![*uncertainty]),
                };
                let query_radius = feature_query_radius(no_coat, broad_phase_keepout);
                for neighbor_index in
                    pad_index.near_circle(no_coat.try_location_f64()?, query_radius)
                {
                    candidate_count += 1;
                    let neighbor = pads[neighbor_index];
                    if std::ptr::eq(no_coat, neighbor) {
                        continue;
                    }
                    if no_coat.layer != neighbor.layer {
                        continue;
                    }
                    if !regiones_within_clearance(
                        &no_coat.region,
                        &neighbor.region,
                        broad_phase_keepout,
                    ) {
                        continue;
                    }
                    if no_coat.net.is_some() && no_coat.net == neighbor.net {
                        continue;
                    }
                    exact_pair_count += 1;
                    let overlap = match intersection_for_check(
                        &keepout_region,
                        &neighbor.region,
                        "conformal-coating-keepout-readiness",
                        vec![no_coat.layer.clone()],
                    ) {
                        Ok(overlap) => overlap,
                        Err(uncertainty) => return Ok(vec![*uncertainty]),
                    };
                    let shapes =
                        multipolygon_to_shapes_scalar(&overlap.try_to_multipolygon()?, min_area)?;
                    if shapes.is_empty() {
                        continue;
                    }

                    violations.push(Violation::new(
                "conformal-coating-keepout-readiness",
                Severity::Warning,
                vec![no_coat.layer.clone()],
                None,
                shapes,
                vec![
                    no_coat.try_location_f64()?,
                    neighbor.try_location_f64()?,
                ],
                Some(format!(
                    "likely no-coat feature {:?} has neighboring pad {:?} inside coating keepout {keepout:#.6}; review conformal-coating mask, cleanliness, and contact/test access",
                    no_coat.net,
                    neighbor.net,
                    keepout = keepout.diagnostic_decimal(-256),
                )),
            ));
                }
            }

            log::trace!(
                "conformal-coating keepout readiness: source={} no_coat_features={} pads={} buckets={} candidate_pairs={} exact_pairs={} keepout={keepout:#.6} violations={}",
                board.source,
                no_coat_features.len(),
                pads.len(),
                pad_index.bucket_count(),
                candidate_count,
                exact_pair_count,
                violations.len(),
                keepout = keepout.diagnostic_decimal(-256),
            );
            debug_assert!(exact_pair_count <= candidate_count);

            Ok(violations)
        },
    )
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

fn feature_query_radius(feature: &CopperFeature, clearance: f64) -> Option<f64> {
    super::spatial::feature_span(feature).map(|span| (span / 2.0 + clearance).next_up())
}

struct FeatureGridIndex<'a> {
    features: &'a [&'a CopperFeature],
    index: super::spatial::CopperSpatialIndex<'a>,
}

impl<'a> FeatureGridIndex<'a> {
    fn new(features: &'a [&'a CopperFeature], clearance: f64) -> Self {
        Self {
            features,
            index: super::spatial::CopperSpatialIndex::new(features, clearance),
        }
    }
    fn bucket_count(&self) -> usize {
        self.index.bucket_count()
    }
    fn near_circle(&self, center: [f64; 2], radius: impl Into<Option<f64>>) -> Vec<usize> {
        match radius.into() {
            Some(radius) => self.index.all_layers_near_circle(center, radius),
            None => (0..self.features.len()).collect(),
        }
    }
}

fn same_layer_feature_candidate_pairs(
    features: &[&CopperFeature],
    clearance: f64,
) -> Vec<(usize, usize)> {
    let index = super::spatial::CopperSpatialIndex::new(features, clearance);
    features
        .iter()
        .enumerate()
        .flat_map(|(left, feature)| {
            index
                .same_layer_near_feature(feature, clearance)
                .into_iter()
                .filter(move |right| *right > left)
                .map(move |right| (left, right))
        })
        .collect()
}

fn regiones_within_clearance(left: &PcbRegion, right: &PcbRegion, clearance: f64) -> bool {
    let Some(left_bounds) = left.conservative_bounds() else {
        return true;
    };
    let Some(right_bounds) = right.conservative_bounds() else {
        return true;
    };

    // AABB broad phase before exact segment/polygon distance.
    left_bounds.min().x - clearance <= right_bounds.max().x
        && left_bounds.max().x + clearance >= right_bounds.min().x
        && left_bounds.min().y - clearance <= right_bounds.max().y
        && left_bounds.max().y + clearance >= right_bounds.min().y
}

fn feature_may_touch_circle(feature: &CopperFeature, center: [f64; 2], radius: f64) -> bool {
    let Some(bounds) = feature.region.conservative_bounds() else {
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

    let Some(bounds) = feature.region.conservative_bounds() else {
        return false;
    };
    let width = bounds.max().x - bounds.min().x;
    let height = bounds.max().y - bounds.min().y;
    let min_dimension = width.min(height);
    let max_dimension = width.max(height);

    min_dimension >= 0.5 && max_dimension <= 2.5 && min_dimension / max_dimension >= 0.75
}

fn scalar_broad_phase_radius(value: &Scalar) -> f64 {
    let projected = value
        .to_f64_lossy()
        .expect("assembly broad-phase radius must fit the finite compatibility index");
    if projected > 0.0 {
        projected.next_up()
    } else {
        0.0
    }
}

fn process_drill_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: &Scalar,
    min_area: &Scalar,
    check: &str,
    process_label: &str,
    net_predicate: fn(&str) -> bool,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(check, &[process_label], || {
        let pads = selected_copper_features(board, selected_layers)
            .into_iter()
            .filter(|feature| feature.kind == CopperKind::Pad)
            .filter(|feature| !likely_fiducial(feature))
            .collect::<Vec<_>>();
        let broad_phase_keepout = scalar_broad_phase_radius(keepout);
        let pad_index = FeatureGridIndex::new(&pads, broad_phase_keepout);
        let drills = board
            .drills
            .iter()
            .filter(|drill| drill.plated)
            .filter(|drill| drill.net.as_deref().is_some_and(net_predicate))
            .collect::<Vec<_>>();
        let mut violations = Vec::new();
        let mut candidate_count = 0_usize;
        let mut exact_pair_count = 0_usize;
        log::trace!(
            "{check}: source={} process={} drills={} pads={} buckets={} keepout={keepout:#.6} min_area={min_area:#.9}",
            board.source,
            process_label,
            drills.len(),
            pads.len(),
            pad_index.bucket_count(),
            keepout = keepout.diagnostic_decimal(-256),
            min_area = min_area.diagnostic_decimal(-256),
        );

        for drill in drills {
            let drill_radius = crate::scalar::half(&drill.diameter);
            let keepout_radius = drill_radius + keepout;
            let broad_phase_radius = scalar_broad_phase_radius(&keepout_radius);
            let center = drill.try_location_f64()?;
            let broad_candidates = pad_index.near_circle(center, broad_phase_radius);
            candidate_count += broad_candidates.len();
            let candidates = broad_candidates
                .into_iter()
                .filter(|&index| drill.net.is_none() || drill.net != pads[index].net)
                .filter(|&index| feature_may_touch_circle(pads[index], center, broad_phase_radius))
                .collect::<Vec<_>>();
            if candidates.is_empty() {
                continue;
            }
            let keepout_region = PcbRegion::new(
                crate::geometry::circle_region(
                    &[drill.location[0].clone(), drill.location[1].clone()],
                    &(keepout_radius),
                )?,
                Some(LayerMetadata {
                    name: format!("{process_label} keepout"),
                }),
            );
            for pad_index in candidates {
                let pad = pads[pad_index];
                exact_pair_count += 1;
                let overlap = match intersection_for_check(
                    &keepout_region,
                    &pad.region,
                    check,
                    vec![pad.layer.clone()],
                ) {
                    Ok(overlap) => overlap,
                    Err(uncertainty) => return Ok(vec![*uncertainty]),
                };
                let shapes =
                    multipolygon_to_shapes_scalar(&overlap.try_to_multipolygon()?, min_area)?;
                if shapes.is_empty() {
                    continue;
                }
                violations.push(Violation::new(
                check,
                Severity::Warning,
                vec![pad.layer.clone()],
                None,
                shapes,
                vec![drill.try_location_f64()?, pad.try_location_f64()?],
                Some(format!(
                    "likely {process_label} feature on net {:?} is within keepout {keepout:#.6} of neighboring pad {:?}; review insertion tooling, component keepout, and assembly drawing notes",
                    drill.net,
                    pad.net,
                    keepout = keepout.diagnostic_decimal(-256),
                )),
            ));
            }
        }

        log::trace!(
            "{check}: source={} candidate_pairs={} exact_pairs={} violations={}",
            board.source,
            candidate_count,
            exact_pair_count,
            violations.len()
        );
        debug_assert!(exact_pair_count <= candidate_count);

        Ok(violations)
    })
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
    use crate::geometry::{polygons_to_profile, rect_polygon};
    use crate::ipc356::{Ipc356AccessSide, Ipc356FeatureType, Ipc356Point, Ipc356Soldermask};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};

    #[test]
    fn pad_pair_asymmetry_readiness_reports_mismatched_neighbor_pads() {
        let board = board_with_copper(vec![
            copper_pad("A", [0.0, 0.0], 0.5, 0.5),
            copper_pad("B", [0.7, 0.0], 1.1, 0.5),
        ]);

        let violations = pad_pair_asymmetry_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.3"),
            &crate::scalar::scalar("1.5"),
            &crate::scalar::scalar("2.0"),
        );

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

        for board in [&balanced, &distant, &large_connector] {
            assert!(
                pad_pair_asymmetry_readiness(
                    board,
                    &[],
                    &crate::scalar::scalar("0.3"),
                    &crate::scalar::scalar("1.5"),
                    &crate::scalar::scalar("2.0")
                )
                .is_empty()
            );
        }
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
        let violations = pad_pair_asymmetry_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.3"),
            &crate::scalar::scalar("1.5"),
            &crate::scalar::scalar("2.0"),
        );

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

        let violations = selective_wave_solder_keepout_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

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

        let violations = press_fit_keepout_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.35"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "press-fit-keepout-readiness");
    }

    #[test]
    fn conformal_coating_keepout_reports_contact_neighbor() {
        let board = board_with_copper(vec![
            copper_pad("USB_DP", [0.0, 0.0], 0.4, 0.4),
            copper_pad("SIG", [0.55, 0.0], 0.3, 0.3),
        ]);

        let violations = conformal_coating_keepout_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.3"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
        let violations = conformal_coating_keepout_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.3"),
            &crate::scalar::scalar("1.0e-9"),
        );

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

        assert!(
            selective_wave_solder_keepout_readiness(
                &board,
                &[],
                &crate::scalar::scalar("0.25"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
        assert!(
            press_fit_keepout_readiness(
                &board,
                &[],
                &crate::scalar::scalar("0.35"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
        assert!(
            conformal_coating_keepout_readiness(
                &board,
                &[],
                &crate::scalar::scalar("0.3"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn mouse_bite_readiness_culls_sparse_drill_rows() {
        let mut drills = (0..1_000)
            .flat_map(|index| {
                let x = index as f64 * 10.0;
                [
                    DrillFeature {
                        location: [
                            crate::geometry::exact_real(x),
                            crate::geometry::exact_real(0.0),
                        ],
                        diameter: crate::scalar::scalar("0.30"),
                        net: None,
                        plated: false,
                    },
                    DrillFeature {
                        location: [
                            crate::geometry::exact_real(x + 0.70),
                            crate::geometry::exact_real(0.0),
                        ],
                        diameter: crate::scalar::scalar("0.30"),
                        net: None,
                        plated: false,
                    },
                ]
            })
            .collect::<Vec<_>>();
        drills.push(DrillFeature {
            location: [
                crate::geometry::exact_real(50_000.0),
                crate::geometry::exact_real(0.0),
            ],
            diameter: crate::scalar::scalar("0.30"),
            net: None,
            plated: false,
        });

        let start = std::time::Instant::now();
        let violations = mouse_bite_readiness(
            &board_with_copper(Vec::new()),
            &drills,
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("0.50"),
            &crate::scalar::scalar("0.40"),
            &crate::scalar::scalar("1.20"),
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
        let selective = selective_wave_solder_keepout_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );
        let press = press_fit_keepout_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.35"),
            &crate::scalar::scalar("1.0e-9"),
        );

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

        let violations = component_hole_clearance_readiness(
            &board,
            &[],
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
        let violations = component_hole_clearance_readiness(
            &board,
            &[],
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

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

        let violations = component_spacing_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("0.5"),
        );

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
        assert!(
            component_spacing_readiness(
                &distant,
                &[],
                &crate::scalar::scalar("0.25"),
                &crate::scalar::scalar("0.5")
            )
            .is_empty()
        );

        let small = board_with_copper(vec![
            copper_pad("R1", [0.0, 0.0], 0.3, 0.3),
            copper_pad("R2", [0.4, 0.0], 0.3, 0.3),
        ]);
        assert!(
            component_spacing_readiness(
                &small,
                &[],
                &crate::scalar::scalar("0.25"),
                &crate::scalar::scalar("0.5")
            )
            .is_empty()
        );

        let other_layer = board_with_copper(vec![
            copper_pad("J1", [0.0, 0.0], 1.0, 0.8),
            copper_pad_on_layer("B.Cu", "J2", [1.15, 0.0], 1.0, 0.8),
        ]);
        assert!(
            component_spacing_readiness(
                &other_layer,
                &[],
                &crate::scalar::scalar("0.25"),
                &crate::scalar::scalar("0.5")
            )
            .is_empty()
        );

        let selected_out = board_with_copper(vec![
            copper_pad_on_layer("B.Cu", "J1", [0.0, 0.0], 1.0, 0.8),
            copper_pad_on_layer("B.Cu", "J2", [1.15, 0.0], 1.0, 0.8),
        ]);
        assert!(
            component_spacing_readiness(
                &selected_out,
                &["F.Cu".to_string()],
                &crate::scalar::scalar("0.25"),
                &crate::scalar::scalar("0.5")
            )
            .is_empty()
        );
    }

    #[test]
    fn connector_rework_clearance_readiness_reports_tight_neighboring_pad() {
        let board = board_with_copper(vec![
            copper_pad("USB_DP", [0.0, 0.0], 1.0, 0.8),
            copper_pad("SIG", [0.82, 0.0], 0.25, 0.25),
            copper_pad("USB_DM", [0.0, 1.0], 1.0, 0.8),
        ]);

        let violations = connector_rework_clearance_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("0.5"),
        );

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
        let violations = connector_rework_clearance_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("0.5"),
        );

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
        let violations = component_spacing_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("0.5"),
        );

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

        let violations = fiducial_keepout_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
        assert!(
            fiducial_keepout_readiness(
                &clear,
                &[],
                &crate::scalar::scalar("0.25"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );

        let other_layer = board_with_copper(vec![
            fiducial("F.Cu", [0.0, 0.0], 0.8),
            copper_pad_on_layer("B.Cu", "SIG", [0.75, 0.0], 0.25, 0.25),
        ]);
        assert!(
            fiducial_keepout_readiness(
                &other_layer,
                &[],
                &crate::scalar::scalar("0.25"),
                &crate::scalar::scalar("1.0e-9"),
            )
            .is_empty()
        );

        let selected_out = board_with_copper(vec![fiducial("B.Cu", [0.0, 0.0], 0.8)]);
        assert!(
            fiducial_keepout_readiness(
                &selected_out,
                &["F.Cu".to_string()],
                &crate::scalar::scalar("0.25"),
                &crate::scalar::scalar("1.0e-9")
            )
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
        let violations = fiducial_keepout_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

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

        let violations = testpoint_copper_clearance_readiness(
            &board,
            &[point],
            &[],
            &crate::scalar::scalar("0.4"),
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
            testpoint_copper_clearance_readiness(
                &same_net,
                &[point],
                &[],
                &crate::scalar::scalar("0.4"),
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );

        let far = board_with_copper(vec![
            copper_pad("TP_NET", [0.0, 0.0], 0.4, 0.4),
            copper_pad("OTHER", [2.0, 0.0], 0.25, 0.25),
        ]);
        let point = ipc_point("TP_NET", [0.0, 0.0], Some(0.4));
        assert!(
            testpoint_copper_clearance_readiness(
                &far,
                &[point],
                &[],
                &crate::scalar::scalar("0.4"),
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
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
                &crate::scalar::scalar("0.4"),
                &crate::scalar::scalar("0.1"),
                &crate::scalar::scalar("1.0e-9")
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
        let violations = testpoint_copper_clearance_readiness(
            &board,
            &[point],
            &[],
            &crate::scalar::scalar("0.4"),
            &crate::scalar::scalar("0.1"),
            &crate::scalar::scalar("1.0e-9"),
        );

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

        let violations = testpoint_accessibility_readiness(
            &board,
            &points,
            &crate::scalar::scalar("0.4"),
            &crate::scalar::scalar("0.35"),
            &crate::scalar::scalar("0.5"),
        );

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
        let violations = testpoint_accessibility_readiness(
            &board,
            &points,
            &crate::scalar::scalar("0.4"),
            &crate::scalar::scalar("0.35"),
            &crate::scalar::scalar("0.5"),
        );

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
        let violations = testpoint_accessibility_readiness(
            &board,
            &[point],
            &crate::scalar::scalar("0.4"),
            &crate::scalar::scalar("0.35"),
            &crate::scalar::scalar("0.5"),
        );

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
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            region: polygons_to_profile(
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
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            region: polygons_to_profile(
                vec![rect_polygon(location, [diameter, diameter], 0.0)],
                Some(LayerMetadata {
                    name: "fiducial".to_string(),
                }),
            ),
        }
    }

    fn plated_drill(net: &str, location: [f64; 2], diameter: f64) -> DrillFeature {
        DrillFeature {
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            diameter: crate::geometry::exact_real(diameter),
            net: Some(net.to_string()),
            plated: true,
        }
    }

    fn npth_drill(location: [f64; 2], diameter: f64) -> DrillFeature {
        DrillFeature {
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            diameter: crate::geometry::exact_real(diameter),
            net: None,
            plated: false,
        }
    }

    fn ipc_point(net: &str, location: [f64; 2], diameter: Option<f64>) -> Ipc356Point {
        Ipc356Point {
            net: net.to_string(),
            reference: Some("TP1".to_string()),
            pin: Some("1".to_string()),
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

    fn accessible_ipc_point(net: &str, location: [f64; 2], diameter: f64) -> Ipc356Point {
        Ipc356Point {
            net: net.to_string(),
            reference: Some(net.to_string()),
            pin: Some("1".to_string()),
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            diameter: Some(crate::geometry::exact_real(diameter)),
            access_side: Some(Ipc356AccessSide::Top),
            feature_type: Some(Ipc356FeatureType::Smd),
            soldermask: Some(Ipc356Soldermask::Open),
        }
    }
}
