//! Native drill/copper authored-center correspondence for plating intent.

use super::selected_copper_features;
use crate::Scalar;
use crate::checks::{measurements, spatial::NativePointSpatialIndex};
use crate::kicad::{BoardModel, CopperKind};
use crate::report::{Severity, Violation};

/// Review plated/NPTH drill intent against parsed copper centers.
/// Plated holes require same-net pad/via evidence when a drill net is known.
/// NPTH holes use their validated native radius plus tolerance. Missing radius
/// certificates or required report coordinates remain typed uncertainty.
pub fn plating_intent(
    board: &BoardModel,
    selected_layers: &[String],
    tolerance: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("plating-intent", &[&board.source], || {
        crate::operations::evaluate("plating-intent", Some(&board.source), || {
            let features = selected_copper_features(board, selected_layers);
            let index = NativePointSpatialIndex::new(
                features.iter().map(|feature| &feature.location),
                tolerance,
            );
            let mut violations = Vec::new();
            let mut candidate_hits = 0;
            for drill in &board.drills {
                let search_radius = if drill.plated {
                    tolerance.clone()
                } else {
                    measurements::drill_radius(&drill.diameter)? + tolerance
                };
                let candidates = index.candidates_near_with_radius(&drill.location, &search_radius);
                candidate_hits += candidates.len();
                let has_copper = measurements::any_match(candidates, |other| {
                    let feature = features[other];
                    if drill.plated
                        && (!matches!(feature.kind, CopperKind::Pad | CopperKind::Via)
                            || (drill.net.is_some() && feature.net != drill.net))
                    {
                        return Ok(false);
                    }
                    measurements::points_within_radius(
                        &feature.location,
                        &drill.location,
                        &search_radius,
                    )
                })?;
                if drill.plated == has_copper {
                    continue;
                }
                let (layer, message) = if drill.plated {
                    (
                        "KiCad drills",
                        "plated drill has no nearby same-net pad or via copper",
                    )
                } else {
                    (
                        "KiCad NPTH drills",
                        "non-plated drill has nearby copper that may imply plated-hole intent",
                    )
                };
                violations.push(Violation::new(
                    "plating-intent",
                    Severity::Warning,
                    vec![layer.into()],
                    None,
                    Vec::new(),
                    vec![measurements::report_point(&drill.location)?],
                    Some(message.into()),
                ));
            }
            log::trace!(
                "plating intent: source={} drills={} copper_features={} copper_buckets={} candidate_hits={} selected_layers={} tolerance={tolerance:.6} violations={}",
                board.source,
                board.drills.len(),
                features.len(),
                index.bucket_count(),
                candidate_hits,
                selected_layers.len(),
                violations.len(),
            );
            Ok(violations)
        })
    })
}
