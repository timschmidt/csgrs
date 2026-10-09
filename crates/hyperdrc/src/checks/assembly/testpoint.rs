//! Native IPC fixture coverage, accessibility and copper keepout review.

use super::{looks_testpoint_required_net, normalize_net, selected_copper_features};
use crate::Scalar;
use crate::checks::{
    distance::point_region_boundary_within, measurements, spatial::NativeRegionSpatialIndex,
};
use crate::ipc356::Ipc356Point;
use crate::kicad::BoardModel;
use crate::report::{Severity, Violation};
use std::collections::{BTreeMap, BTreeSet};

mod access;
mod probe;
pub use access::testpoint_accessibility_readiness;

/// Compare critical KiCad net names against normalized IPC test records.
/// Covered nets require no geometric projection. Only the first three authored
/// markers for each missing net are required report evidence. This is net-name
/// coverage, not a geometric assertion that a particular pad is accessible.
pub fn testpoint_coverage_readiness(
    board: &BoardModel,
    points: &[Ipc356Point],
    selected_layers: &[String],
) -> Vec<Violation> {
    crate::checks::run_geometry_check("testpoint-coverage-readiness", &[&board.source], || {
        crate::operations::evaluate("testpoint-coverage-readiness", Some(&board.source), || {
            let covered = points
                .iter()
                .map(|point| normalize_net(&point.net))
                .collect::<BTreeSet<_>>();
            let mut missing = BTreeMap::<&str, Vec<&[Scalar; 2]>>::new();
            for feature in selected_copper_features(board, selected_layers) {
                let Some(net) = feature.net.as_deref() else {
                    continue;
                };
                if !looks_testpoint_required_net(net) || covered.contains(&normalize_net(net)) {
                    continue;
                }
                let markers = missing.entry(net).or_default();
                if markers.len() < 3 {
                    markers.push(&feature.location);
                }
            }
            missing.into_iter().map(|(net, points)| Ok(Violation::new(
                "testpoint-coverage-readiness", Severity::Warning, vec![net.to_owned()], None, Vec::new(),
                points.into_iter().map(measurements::report_point).collect::<Result<_, _>>()?,
                Some(format!("critical net {net:?} has parsed KiCad copper but no matching IPC-D-356 test record")),
            ))).collect()
        })
    })
}

/// Review native circular probe envelopes against unrelated selected copper.
/// Candidate scheduling uses native shape enclosures, independently of authored
/// markers. Required classification/distance/marker failures retain uncertainty;
/// unknown comparisons cannot also emit a claimed copper intersection.
pub fn testpoint_copper_clearance_readiness(
    board: &BoardModel,
    points: &[Ipc356Point],
    selected_layers: &[String],
    minimum_diameter: &Scalar,
    clearance: &Scalar,
    _min_area: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "testpoint-copper-clearance-readiness",
        &[&board.source],
        || {
            crate::operations::evaluate(
                "testpoint-copper-clearance-readiness",
                Some(&board.source),
                || {
                    let copper = selected_copper_features(board, selected_layers);
                    let index =
                        NativeRegionSpatialIndex::new(copper.iter().map(|f| &f.region), clearance);
                    let mut violations = Vec::new();
                    let mut candidates_count = 0;
                    let mut exact_count = 0;
                    for point in points {
                        let point_net = normalize_net(&point.net);
                        let mut candidates =
                            probe::candidate_radii(point, minimum_diameter, clearance)
                                .iter()
                                .flat_map(|radius| {
                                    index.candidates_near_point(&point.location, radius)
                                })
                                .collect::<Vec<_>>();
                        candidates.sort_unstable();
                        candidates.dedup();
                        candidates_count += candidates.len();
                        let mut radii = None;
                        for other in candidates {
                            let feature = copper[other];
                            if feature
                                .net
                                .as_deref()
                                .is_some_and(|net| normalize_net(net) == point_net)
                            {
                                continue;
                            }
                            if feature.region.try_is_empty()? {
                                continue;
                            }
                            let radii = match &radii {
                                Some(radii) => radii,
                                None => radii.insert(probe::radii(point, minimum_diameter)?),
                            };
                            exact_count += 1;
                            let inside = feature.region.classify_xy(
                                point.location[0].clone(),
                                point.location[1].clone(),
                            )? != hypercurve::RegionPointLocation::Outside;
                            if !inside
                                && !measurements::any_match(radii, |radius| {
                                    point_region_boundary_within(
                                        &point.location,
                                        &feature.region,
                                        &(radius + clearance),
                                    )
                                })?
                            {
                                continue;
                            }
                            violations.push(Violation::new(
                        "testpoint-copper-clearance-readiness", Severity::Warning, vec![feature.layer.clone(), format!("net:{}", point.net)], None, Vec::new(),
                        vec![measurements::report_point(&point.location)?, measurements::report_point(&feature.location)?],
                        Some(format!("IPC-D-356 testpoint probe keepout {:.6} around net {:?} intersects unrelated KiCad copper {:?}; review fixture short risk and probe clearance", clearance.fixed_decimal(), point.net, feature.net)),
                    ));
                        }
                    }
                    log::trace!(
                        "testpoint copper clearance: source={} buckets={} candidates={} exact_checks={} violations={}",
                        board.source,
                        index.bucket_count(),
                        candidates_count,
                        exact_count,
                        violations.len()
                    );
                    debug_assert!(exact_count <= candidates_count);
                    Ok(violations)
                },
            )
        },
    )
}
