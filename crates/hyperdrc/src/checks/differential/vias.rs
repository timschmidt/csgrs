//! Native center matching for inferred differential-pair via transitions.

use std::collections::{BTreeMap, BTreeSet};

use super::{DifferentialSide, differential_pair_key, looks_ground_net, selected_copper_features};
use crate::Scalar;
use crate::checks::{measurements, spatial::NativePointSpatialIndex};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};

/// Warn when an inferred differential-pair via lacks a nearby opposite-side via.
/// Native center enclosures schedule candidates and checked squared distances
/// decide closed-radius matches. A certified alternative resolves existence;
/// otherwise missing certificates remain uncertainty. This is a placement proxy,
/// not a delay extraction or stackup certification.
pub fn differential_pair_via_proximity_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    maximum_via_pair_gap: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "differential-pair-via-proximity-readiness",
        &[&board.source],
        || {
            crate::operations::evaluate(
                "differential-pair-via-proximity-readiness",
                Some(&board.source),
                || {
                    let features = selected_copper_features(board, selected_layers);
                    let pairs = pair_vias(&features);
                    let inferred_pairs = pairs.len();
                    let mut spatial_buckets = 0;
                    let mut candidate_count = 0;
                    let mut violations = Vec::new();
                    for (pair, usage) in pairs {
                        if usage.positive.locations.is_empty()
                            || usage.negative.locations.is_empty()
                        {
                            continue;
                        }
                        for (label, opposite_label, side, opposite) in [
                            ("positive", "negative", &usage.positive, &usage.negative),
                            ("negative", "positive", &usage.negative, &usage.positive),
                        ] {
                            let index = NativePointSpatialIndex::new(
                                opposite.locations.iter().copied(),
                                maximum_via_pair_gap,
                            );
                            spatial_buckets += index.bucket_count();
                            for location in &side.locations {
                                let candidates = index.candidates_near(location);
                                candidate_count += candidates.len();
                                let nearby = measurements::any_match(candidates, |other| {
                                    measurements::points_within_radius(
                                        location,
                                        opposite.locations[other],
                                        maximum_via_pair_gap,
                                    )
                                })?;
                                if nearby {
                                    continue;
                                }
                                let message = format!(
                                    "likely differential pair {pair} {label}-side via has no parsed {opposite_label}-side via within {maximum_via_pair_gap:.6}; review layer-change symmetry and return-path stitching"
                                );
                                violations.push(Violation::new(
                                    "differential-pair-via-proximity-readiness",
                                    Severity::Warning,
                                    usage.layers(),
                                    None,
                                    Vec::new(),
                                    vec![measurements::report_point(location)?],
                                    Some(message),
                                ));
                            }
                        }
                    }
                    log::trace!(
                        "differential pair via proximity readiness: source={} inferred_pairs={} selected_layers={} spatial_buckets={} candidate_count={} threshold={maximum_via_pair_gap:.6} violations={}",
                        board.source,
                        inferred_pairs,
                        selected_layers.len(),
                        spatial_buckets,
                        candidate_count,
                        violations.len()
                    );
                    Ok(violations)
                },
            )
        },
    )
}

/// Warn when inferred differential-pair transitions lack nearby ground vias.
/// Required native center matching and report projection retain typed failures.
/// The inferred pair and ground names remain readiness heuristics requiring
/// review against schematic, stackup and field-solver evidence.
pub fn differential_pair_via_return_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    stitching_distance: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "differential-pair-via-return-readiness",
        &[&board.source],
        || {
            crate::operations::evaluate(
                "differential-pair-via-return-readiness",
                Some(&board.source),
                || {
                    let features = selected_copper_features(board, selected_layers);
                    let ground = features
                        .iter()
                        .copied()
                        .filter(|feature| {
                            feature.kind == CopperKind::Via
                                && feature.net.as_deref().is_some_and(looks_ground_net)
                        })
                        .collect::<Vec<_>>();
                    let pairs = pair_vias(&features);
                    let inferred_pairs = pairs.len();
                    let index = NativePointSpatialIndex::new(
                        ground.iter().map(|feature| &feature.location),
                        stitching_distance,
                    );
                    let mut violations = Vec::new();
                    let mut candidate_count = 0;
                    for (pair, usage) in pairs {
                        if usage.positive.locations.is_empty()
                            || usage.negative.locations.is_empty()
                        {
                            continue;
                        }
                        for location in usage
                            .positive
                            .locations
                            .iter()
                            .chain(&usage.negative.locations)
                        {
                            let candidates = index.candidates_near(location);
                            candidate_count += candidates.len();
                            let nearby = measurements::any_match(candidates, |other| {
                                measurements::points_within_radius(
                                    location,
                                    &ground[other].location,
                                    stitching_distance,
                                )
                            })?;
                            if nearby {
                                continue;
                            }
                            let message = format!(
                                "likely differential pair {pair} via transition has no parsed ground stitching via within {stitching_distance:.6}; review return-path continuity through the layer change"
                            );
                            violations.push(Violation::new(
                                "differential-pair-via-return-readiness",
                                Severity::Warning,
                                usage.layers(),
                                None,
                                Vec::new(),
                                vec![measurements::report_point(location)?],
                                Some(message),
                            ));
                        }
                    }
                    log::trace!(
                        "differential pair via return readiness: source={} inferred_pairs={} ground_vias={} ground_buckets={} candidate_count={} selected_layers={} threshold={stitching_distance:.6} violations={}",
                        board.source,
                        inferred_pairs,
                        ground.len(),
                        index.bucket_count(),
                        candidate_count,
                        selected_layers.len(),
                        violations.len()
                    );
                    Ok(violations)
                },
            )
        },
    )
}

fn pair_vias<'a>(features: &[&'a CopperFeature]) -> BTreeMap<String, PairViaUse<'a>> {
    let mut pairs = BTreeMap::<String, PairViaUse<'a>>::new();
    for feature in features {
        if feature.kind != CopperKind::Via {
            continue;
        }
        let Some((pair, side)) = feature.net.as_deref().and_then(differential_pair_key) else {
            continue;
        };
        let usage = pairs.entry(pair).or_default();
        let side = match side {
            DifferentialSide::Positive => &mut usage.positive,
            DifferentialSide::Negative => &mut usage.negative,
        };
        side.layers.insert(feature.layer.clone());
        side.locations.push(&feature.location);
    }
    pairs
}

#[derive(Default)]
struct PairViaUse<'a> {
    positive: SideViaUse<'a>,
    negative: SideViaUse<'a>,
}

impl PairViaUse<'_> {
    fn layers(&self) -> Vec<String> {
        self.positive
            .layers
            .union(&self.negative.layers)
            .cloned()
            .collect()
    }
}

#[derive(Default)]
struct SideViaUse<'a> {
    layers: BTreeSet<String>,
    locations: Vec<&'a [Scalar; 2]>,
}
