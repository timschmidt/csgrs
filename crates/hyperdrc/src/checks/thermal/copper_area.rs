//! Authored-center evidence for power-feature heat spreading.

use super::{looks_thermal_or_power_net, selected_copper_features};
use crate::Scalar;
use crate::checks::{measurements, spatial::NativePointSpatialIndex};
use crate::kicad::{BoardModel, CopperKind};
use crate::report::{Severity, Violation};

/// Review likely hot or high-current features for nearby same-net zone centers.
/// This retains the placement proxy; it does not measure copper area or prove
/// temperature rise. Layer/net eligibility precedes checked native matching.
/// Unresolved candidates cannot prove absence; a certified alternative proves
/// proximity without requiring report projections or copper-shape geometry.
pub fn thermal_copper_area_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    search_radius: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("thermal-copper-area-readiness", &[&board.source], || {
        crate::operations::evaluate("thermal-copper-area-readiness", Some(&board.source), || {
            let features = selected_copper_features(board, selected_layers);
            let zones = features
                .iter()
                .copied()
                .filter(|f| f.kind == CopperKind::Zone)
                .filter(|f| f.net.as_deref().is_some_and(looks_thermal_or_power_net))
                .collect::<Vec<_>>();
            let index =
                NativePointSpatialIndex::new(zones.iter().map(|f| &f.location), search_radius);
            let mut violations = Vec::new();
            let mut candidate_count = 0;
            let mut exact_count = 0;
            for feature in features {
                if feature.kind == CopperKind::Zone {
                    continue;
                }
                let Some(net) = feature
                    .net
                    .as_deref()
                    .filter(|net| looks_thermal_or_power_net(net))
                else {
                    continue;
                };
                let candidates = index.candidates_near(&feature.location);
                candidate_count += candidates.len();
                if measurements::any_match(candidates, |other| {
                    let zone = zones[other];
                    if zone.layer != feature.layer || zone.net != feature.net {
                        return Ok(false);
                    }
                    exact_count += 1;
                    measurements::points_within_radius(
                        &feature.location,
                        &zone.location,
                        search_radius,
                    )
                })? {
                    continue;
                }
                violations.push(Violation::new(
                    "thermal-copper-area-readiness", Severity::Warning, vec![feature.layer.clone()], None, Vec::new(),
                    vec![measurements::report_point(&feature.location)?],
                    Some(format!("likely heat or power feature on net {net:?} has no parsed same-net copper zone within {:.6}; review copper area for heat spreading and current return", search_radius.fixed_decimal())),
                ));
            }
            log::trace!(
                "thermal copper-area readiness: source={} zones={} buckets={} candidates={} exact_checks={} violations={}",
                board.source,
                zones.len(),
                index.bucket_count(),
                candidate_count,
                exact_count,
                violations.len()
            );
            debug_assert!(exact_count <= candidate_count);
            Ok(violations)
        })
    })
}
