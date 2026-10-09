//! Native via-center matching for board stitching and power arrays.

use super::{
    looks_chassis_net, looks_ground_net, looks_high_current_net, looks_high_speed_net,
    looks_rf_or_antenna_net, selected_copper_features,
};
use crate::checks::outline::{
    axis_aligned_outline_rect_with_grid, feature_bounds_inside_rect_margin_with_grid,
};
use crate::checks::spatial::{NativePointSpatialIndex, native_radius_upper};
use crate::checks::{difference_for_check, measurements, offset_for_check};
use crate::geometry::{SourceGridFacts, multipolygon_to_shapes_scalar};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};
use crate::{PcbGeometryUncertainty, PcbRegionExt, Scalar};
use std::collections::{BTreeMap, BTreeSet};

fn has_ground(
    index: &NativePointSpatialIndex,
    ground: &[&CopperFeature],
    center: &[Scalar; 2],
    radius: &Scalar,
) -> Result<bool, PcbGeometryUncertainty> {
    measurements::any_match(index.candidates_near(center), |candidate| {
        measurements::points_within_radius(center, &ground[candidate].location, radius)
    })
}

/// Warn about high-current via centers without a distinct same-net neighbor.
/// Native enclosures schedule candidates, and checked squared distances decide
/// the closed pitch boundary. Coincident copies on other layers are not neighbors.
pub fn power_via_array_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    maximum_isolated_pitch: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("power-via-array-readiness", &[&board.source], || {
        crate::operations::evaluate("power-via-array-readiness", Some(&board.source), || {
            let mut nets: BTreeMap<String, ViaArrayUse<'_>> = BTreeMap::new();
            for feature in selected_copper_features(board, selected_layers) {
                let Some(net) = &feature.net else {
                    continue;
                };
                if feature.kind != CopperKind::Via || !looks_high_current_net(net) {
                    continue;
                }
                let usage = nets.entry(net.clone()).or_default();
                usage.layers.insert(feature.layer.clone());
                usage.locations.push(&feature.location);
            }
            let mut violations = Vec::new();
            let mut indexed_nets = 0;
            let mut spatial_buckets = 0;
            let mut candidate_count = 0;
            for (net, usage) in nets {
                if usage.locations.len() < 2 {
                    continue;
                }
                let index = NativePointSpatialIndex::new(
                    usage.locations.iter().copied(),
                    maximum_isolated_pitch,
                );
                indexed_nets += 1;
                spatial_buckets += index.bucket_count();
                let mut isolated = Vec::new();
                for (own_index, location) in usage.locations.iter().enumerate() {
                    let candidates = index.candidates_near(location);
                    candidate_count += candidates.len();
                    let has_neighbor = measurements::any_match(candidates, |other| {
                        if other == own_index
                            || !measurements::points_within_radius(
                                location,
                                usage.locations[other],
                                maximum_isolated_pitch,
                            )?
                        {
                            return Ok(false);
                        }
                        Ok(!measurements::points_within_radius(
                            location,
                            usage.locations[other],
                            &Scalar::zero(),
                        )?)
                    })?;
                    if !has_neighbor {
                        isolated.push(measurements::report_point(location)?);
                    }
                }
                if isolated.is_empty() {
                    continue;
                }
                let message = format!(
                    "likely high-current net {net} has isolated vias farther than {maximum_isolated_pitch:.6} from the rest of the same-net via array"
                );
                violations.push(Violation::new(
                    "power-via-array-readiness",
                    Severity::Warning,
                    usage.layers.into_iter().collect(),
                    None,
                    Vec::new(),
                    isolated,
                    Some(message),
                ));
            }
            log::trace!(
                "power-via array readiness: source={} indexed_nets={} spatial_buckets={} candidate_count={} maximum_isolated_pitch={maximum_isolated_pitch:.6} violations={}",
                board.source,
                indexed_nets,
                spatial_buckets,
                candidate_count,
                violations.len()
            );
            Ok(violations)
        })
    })
}

#[derive(Default)]
struct ViaArrayUse<'a> {
    layers: BTreeSet<String>,
    locations: Vec<&'a [Scalar; 2]>,
}

/// Warn when chassis/shield copper lacks a certified nearby ground via.
/// This native center-distance proxy retains unresolved candidates and remains
/// subject to review against chassis, enclosure and bonding intent.
pub fn chassis_stitching_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    stitching_distance: &Scalar,
) -> Vec<Violation> {
    chassis_stitching_readiness_with_grid(
        board,
        selected_layers,
        stitching_distance,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Run chassis stitching over authoritative native center coordinates.
/// The source-grid argument is retained for API compatibility; the centers
/// already carry their native source values and need no finite lifting step.
pub fn chassis_stitching_readiness_with_grid(
    board: &BoardModel,
    selected_layers: &[String],
    stitching_distance: &Scalar,
    _grid: SourceGridFacts,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("chassis-stitching-readiness", &[&board.source], || {
        crate::operations::evaluate("chassis-stitching-readiness", Some(&board.source), || {
            let features = selected_copper_features(board, selected_layers);
            let ground = features
                .iter()
                .copied()
                .filter(|feature| {
                    feature.kind == CopperKind::Via
                        && feature.net.as_deref().is_some_and(looks_ground_net)
                })
                .collect::<Vec<_>>();
            let index = NativePointSpatialIndex::new(
                ground.iter().map(|feature| &feature.location),
                stitching_distance,
            );
            let mut violations = Vec::new();
            let mut candidate_features = 0;
            let mut stitched_features = 0;
            for feature in features {
                let Some(net) = feature.net.as_deref().filter(|net| looks_chassis_net(net)) else {
                    continue;
                };
                candidate_features += 1;
                if has_ground(&index, &ground, &feature.location, stitching_distance)? {
                    stitched_features += 1;
                    continue;
                }
                let message = format!(
                    "likely chassis or shield net {net} has no parsed ground stitching via within {stitching_distance:.6}; review shield bonding and EMC stitching intent"
                );
                violations.push(Violation::new(
                    "chassis-stitching-readiness",
                    Severity::Warning,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![measurements::report_point(&feature.location)?],
                    Some(message),
                ));
            }
            log::trace!(
                "chassis stitching readiness: source={} candidate_features={} ground_vias={} ground_buckets={} stitched_features={} stitching_distance={stitching_distance:.6} violations={}",
                board.source,
                candidate_features,
                ground.len(),
                index.bucket_count(),
                stitched_features,
                violations.len()
            );
            Ok(violations)
        })
    })
}

/// Warn when high-speed via transitions lack a certified nearby ground via.
/// Native enclosures preserve candidates; unresolved matching remains a typed
/// uncertainty unless another candidate proves the required return-path proxy.
pub fn return_path_readiness(
    board: &BoardModel,
    stitching_distance: &Scalar,
    selected_layers: &[String],
) -> Vec<Violation> {
    crate::checks::run_geometry_check("return-path-readiness", &[&board.source], || {
        crate::operations::evaluate("return-path-readiness", Some(&board.source), || {
            let features = selected_copper_features(board, selected_layers);
            let ground = features
                .iter()
                .copied()
                .filter(|feature| {
                    feature.kind == CopperKind::Via
                        && feature.net.as_deref().is_some_and(looks_ground_net)
                })
                .collect::<Vec<_>>();
            let index = NativePointSpatialIndex::new(
                ground.iter().map(|feature| &feature.location),
                stitching_distance,
            );
            let mut violations = Vec::new();
            let mut signal_vias = 0;
            let mut stitched_vias = 0;
            for via in features.into_iter().filter(|feature| {
                feature.kind == CopperKind::Via
                    && feature.net.as_deref().is_some_and(looks_high_speed_net)
            }) {
                signal_vias += 1;
                if has_ground(&index, &ground, &via.location, stitching_distance)? {
                    stitched_vias += 1;
                    continue;
                }
                let message = format!(
                    "likely high-speed net {:?} changes layers without a parsed ground stitching via within {stitching_distance:.6}",
                    via.net
                );
                violations.push(Violation::new(
                    "return-path-readiness",
                    Severity::Warning,
                    vec![via.layer.clone()],
                    None,
                    Vec::new(),
                    vec![measurements::report_point(&via.location)?],
                    Some(message),
                ));
            }
            log::trace!(
                "return-path readiness: source={} signal_vias={} ground_vias={} ground_buckets={} stitched_vias={} stitching_distance={stitching_distance:.6} violations={}",
                board.source,
                signal_vias,
                ground.len(),
                index.bucket_count(),
                stitched_vias,
                violations.len()
            );
            Ok(violations)
        })
    })
}

/// Warn when high-speed or RF/antenna nets near the edge have no nearby ground
/// stitching via for intended return-path reinforcement.
///
/// Ground-stitch centers use a point-grid broad phase before the exact center-
/// radius predicate. The result remains a readiness proxy: board-edge return-current
/// quality still needs stackup, chassis, and enclosure review.
pub fn edge_stitching_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: &Scalar,
    stitching_distance: &Scalar,
    min_area: &Scalar,
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
/// predicates before allowing any candidate to skip slower geometry. Authored
/// center coordinates remain native through candidate selection and comparison.
pub fn edge_stitching_readiness_with_grid(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: &Scalar,
    stitching_distance: &Scalar,
    min_area: &Scalar,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("edge-stitching-readiness", &[&board.source], || {
        crate::operations::evaluate("edge-stitching-readiness", Some(&board.source), || {
            let Some(outline) = &board.board_outline else {
                return Ok(Vec::new());
            };
            let outline_rect = axis_aligned_outline_rect_with_grid(outline, grid);
            let broad_phase_edge_clearance = native_radius_upper(edge_clearance);
            let allowed = match offset_for_check(
                outline,
                -edge_clearance.clone(),
                "edge-stitching-readiness",
                vec!["KiCad Edge.Cuts".into()],
            ) {
                Ok(allowed) => allowed,
                Err(uncertainty) => return Ok(vec![*uncertainty]),
            };
            let features = selected_copper_features(board, selected_layers);
            let ground_vias = features
                .iter()
                .copied()
                .filter(|feature| feature.kind == CopperKind::Via)
                .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
                .collect::<Vec<_>>();
            let ground_index = NativePointSpatialIndex::new(
                ground_vias.iter().map(|feature| &feature.location),
                stitching_distance,
            );

            let mut violations = Vec::new();
            let mut candidate_features = 0usize;
            let mut skipped_rect_inside = 0usize;
            let mut stitched_features = 0usize;
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
                    feature_bounds_inside_rect_margin_with_grid(
                        feature,
                        rect,
                        broad_phase_edge_clearance,
                        grid,
                    )
                }) {
                    skipped_rect_inside += 1;
                    continue;
                }

                let intrusion = match difference_for_check(
                    &feature.region,
                    &allowed,
                    "edge-stitching-readiness",
                    vec![feature.layer.clone(), "KiCad Edge.Cuts".into()],
                ) {
                    Ok(intrusion) => intrusion,
                    Err(uncertainty) => return Ok(vec![*uncertainty]),
                };
                let shapes =
                    multipolygon_to_shapes_scalar(&intrusion.try_to_multipolygon()?, min_area)?;
                let has_edge_intrusion = !shapes.is_empty()
                    || (crate::checks::boundaries_within_for_check(
                        &feature.region,
                        outline,
                        edge_clearance,
                        "edge-stitching-readiness",
                        &[&feature.layer],
                        &mut violations,
                    ) == Some(true));
                if !has_edge_intrusion {
                    continue;
                }

                let has_stitch = has_ground(
                    &ground_index,
                    &ground_vias,
                    &feature.location,
                    stitching_distance,
                )?;
                stitched_features += usize::from(has_stitch);
                if has_stitch {
                    continue;
                }

                let message = format!(
                    "likely high-speed or RF net {net} is near board edge without nearby ground stitch vias within {stitching_distance:.6}"
                );
                violations.push(Violation::new(
                    "edge-stitching-readiness",
                    Severity::Warning,
                    vec![feature.layer.clone(), "KiCad Edge.Cuts".to_string()],
                    None,
                    shapes,
                    vec![measurements::report_point(&feature.location)?],
                    Some(message),
                ));
            }

            log::trace!(
                "edge stitching readiness: source={} candidate_features={} ground_vias={} ground_buckets={} stitched_features={} outline_fast_path={} skipped_rect_inside={} edge_clearance={edge_clearance:.6} stitching_distance={stitching_distance:.6} violations={}",
                board.source,
                candidate_features,
                ground_vias.len(),
                ground_index.bucket_count(),
                stitched_features,
                outline_rect.is_some(),
                skipped_rect_inside,
                violations.len()
            );

            Ok(violations)
        })
    })
}
