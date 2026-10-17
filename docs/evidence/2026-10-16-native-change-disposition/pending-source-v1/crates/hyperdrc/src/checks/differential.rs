//! Differential-pair readiness checks.
//!
//! These checks use net-name and copper-geometry proxies for differential-pair
//! intent. They complement exact net-class constraints by making common layout
//! hazards visible even when the project has not declared explicit pair rules.
//!
//! Reliability note: inferred pair names and polygon distances are suspect for
//! release-blocking decisions. Verify findings against schematic constraints,
//! field-solver results, stackup impedance targets, and the fabricator rule set.

use std::collections::{BTreeMap, BTreeSet};

use super::spatial::CopperSpatialIndex;

mod vias;
use crate::geometry::Rect;
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};
use crate::{PcbRegionExt, Scalar};
pub use vias::{differential_pair_via_proximity_readiness, differential_pair_via_return_readiness};

/// Warn when inferred differential-pair segment widths look narrow or unbalanced.
///
/// This check complements generic copper-width checks by keeping differential
/// pair impedance symmetry visible in the default suite. It estimates segment
/// width from the minimum bounding dimension of parsed segment envelopes, so it
/// is a readiness proxy rather than a controlled-impedance calculation.
/// IPC-2221B treats trace width and spacing as board-design constraints.
pub fn differential_pair_width_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_segment_width: &Scalar,
    maximum_side_width_delta: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "differential-pair-width-readiness",
        &[&board.source],
        || {
            let mut pairs = BTreeMap::<String, PairWidthUse>::new();
            let mut violations = Vec::new();

            for feature in selected_copper_features(board, selected_layers) {
                if feature.kind != CopperKind::Segment {
                    continue;
                }
                let Some(net) = &feature.net else {
                    continue;
                };
                let Some((pair, side)) = differential_pair_key(net) else {
                    continue;
                };
                let Some(Some(estimated_width)) = crate::checks::geometry_result_for_check(
                    super::minimum_bounding_dimension(&feature.region),
                    "differential-pair-width-readiness",
                    &[&feature.layer],
                    &mut violations,
                ) else {
                    continue;
                };
                if crate::scalar::le(&estimated_width, &Scalar::zero()) {
                    continue;
                }

                let side_use = pairs.entry(pair.clone()).or_default().side_mut(side);
                side_use.layers.insert(feature.layer.clone());
                side_use.locations.push(feature.try_location_f64()?);
                side_use.widths.push(estimated_width.clone());

                if crate::scalar::lt(&estimated_width, minimum_segment_width) {
                    violations.push(Violation::new(
                    "differential-pair-width-readiness",
                    Severity::Warning,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![feature.try_location_f64()?],
                    Some(format!(
                        "likely differential pair {pair} segment on net {net} has approximate width {estimated_width:.6}, below review threshold {minimum_segment_width:.6}; review impedance, neck-down, and fabricator trace-width rules"
                    )),
                ));
                }
            }
            log::trace!(
                "differential pair width readiness: source={} inferred_pairs={} selected_layers={} min_width={:.6} max_delta={:.6}",
                board.source,
                pairs.len(),
                selected_layers.len(),
                minimum_segment_width,
                maximum_side_width_delta
            );

            for (pair, usage) in pairs {
                let Some(positive_width) = usage.positive.minimum_width() else {
                    continue;
                };
                let Some(negative_width) = usage.negative.minimum_width() else {
                    continue;
                };
                let delta = (&positive_width - &negative_width).abs();
                if crate::scalar::le(&delta, maximum_side_width_delta) {
                    continue;
                }

                violations.push(Violation::new(
                "differential-pair-width-readiness",
                Severity::Warning,
                usage.layers(),
                None,
                Vec::new(),
                usage.locations(),
                Some(format!(
                    "likely differential pair {pair} has approximate side-width delta {delta:.6}, above balance threshold {maximum_side_width_delta:.6}; review width matching, neck-downs, and impedance constraints"
                )),
            ));
            }

            Ok(violations)
        },
    )
}

/// Warn when an inferred differential-pair neck-down is both narrow and long.
///
/// The generic width check reports every narrow inferred pair segment. This
/// companion focuses on the subset most likely to disturb impedance over a
/// meaningful distance: parsed segment envelopes whose estimated width is below
/// the pair-width threshold and whose estimated length exceeds the allowed
/// neck-down length. IPC-2221B treats conductor geometry as a design constraint.
/// This is envelope-based readiness metadata, not impedance solving.
pub fn differential_pair_neckdown_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_segment_width: &Scalar,
    maximum_neckdown_length: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "differential-pair-neckdown-readiness",
        &[&board.source],
        || {
            let mut candidate_count = 0usize;
            let mut violations = Vec::new();
            for feature in selected_copper_features(board, selected_layers) {
                if feature.kind != CopperKind::Segment {
                    continue;
                }
                let Some(net) = &feature.net else {
                    continue;
                };
                let Some((pair, _side)) = differential_pair_key(net) else {
                    continue;
                };
                let Some(Some(estimated_width)) = crate::checks::geometry_result_for_check(
                    super::minimum_bounding_dimension(&feature.region),
                    "differential-pair-neckdown-readiness",
                    &[&feature.layer],
                    &mut violations,
                ) else {
                    continue;
                };
                let Some(Some(estimated_length)) = crate::checks::geometry_result_for_check(
                    estimated_segment_length(feature),
                    "differential-pair-neckdown-readiness",
                    &[&feature.layer],
                    &mut violations,
                ) else {
                    continue;
                };
                if crate::scalar::le(&estimated_width, &Scalar::zero())
                    || crate::scalar::le(&estimated_length, &Scalar::zero())
                {
                    continue;
                }
                if crate::scalar::ge(&estimated_width, minimum_segment_width) {
                    continue;
                }
                candidate_count += 1;
                if crate::scalar::le(&estimated_length, maximum_neckdown_length) {
                    continue;
                }

                violations.push(Violation::new(
                "differential-pair-neckdown-readiness",
                Severity::Warning,
                vec![feature.layer.clone()],
                None,
                Vec::new(),
                vec![feature.try_location_f64()?],
                Some(format!(
                    "likely differential pair {pair} segment on net {net} has approximate neck-down width {estimated_width:.6} and length {estimated_length:.6}, above allowed neck-down length {maximum_neckdown_length:.6}; review impedance discontinuity and escape routing"
                )),
            ));
            }
            log::trace!(
                "differential pair neckdown readiness: source={} narrow_candidates={} selected_layers={} min_width={:.6} max_length={:.6}",
                board.source,
                candidate_count,
                selected_layers.len(),
                minimum_segment_width,
                maximum_neckdown_length
            );

            Ok(violations)
        },
    )
}

/// Warn when inferred differential-pair sides have mismatched parsed length.
///
/// This is a default-suite companion to explicit `net_classes.max_pair_skew`.
/// It uses the maximum bounding dimension of parsed segment envelopes as a
/// conservative length proxy rather than reconstructing full routed topology.
/// IPC-2221B frames impedance and high-speed routing as design constraints.
pub fn differential_pair_skew_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    maximum_pair_skew: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("differential-pair-skew-readiness", &[&board.source], || {
        let mut violations = Vec::new();
        let mut pairs = BTreeMap::<String, PairLengthUse>::new();
        for feature in selected_copper_features(board, selected_layers) {
            if feature.kind != CopperKind::Segment {
                continue;
            }
            let Some(net) = &feature.net else {
                continue;
            };
            let Some((pair, side)) = differential_pair_key(net) else {
                continue;
            };
            let Some(Some(estimated_length)) = crate::checks::geometry_result_for_check(
                estimated_segment_length(feature),
                "differential-pair-skew-readiness",
                &[&feature.layer],
                &mut violations,
            ) else {
                continue;
            };
            if crate::scalar::le(&estimated_length, &Scalar::zero()) {
                continue;
            }

            let side_use = pairs.entry(pair).or_default().side_mut(side);
            side_use.layers.insert(feature.layer.clone());
            side_use.locations.push(feature.try_location_f64()?);
            side_use.estimated_length += &estimated_length;
        }
        log::trace!(
            "differential pair skew readiness: source={} inferred_pairs={} selected_layers={} threshold={:.6}",
            board.source,
            pairs.len(),
            selected_layers.len(),
            maximum_pair_skew
        );

        for (pair, usage) in pairs {
            if crate::scalar::le(&usage.positive.estimated_length, &Scalar::zero())
                || crate::scalar::le(&usage.negative.estimated_length, &Scalar::zero())
            {
                continue;
            }
            let skew = (&usage.positive.estimated_length - &usage.negative.estimated_length).abs();
            if crate::scalar::le(&skew, maximum_pair_skew) {
                continue;
            }

            violations.push(Violation::new(
                "differential-pair-skew-readiness",
                Severity::Warning,
                usage.layers(),
                None,
                Vec::new(),
                usage.locations(),
                Some(format!(
                    "likely differential pair {pair} has approximate parsed segment-length skew {skew:.6}, above review threshold {maximum_pair_skew:.6}; review length matching, meanders, and explicit pair constraints"
                )),
            ));
        }

        Ok(violations)
    })
}

/// Warn when two likely differential pairs run too close on the same layer.
///
/// IPC-2221B treats conductor spacing as a board-level design constraint.
/// Pair-to-pair coupling deserves separate review from simple same-net
/// clearance. This readiness check is deliberately
/// conservative: it infers pair membership from common suffixes, performs a
/// shared copper spatial broad phase, then measures exact polygon boundary
/// distance for nearby features, keeping sparse
/// differential-pair fields from devolving into all-pairs comparisons.
pub fn differential_pair_to_pair_spacing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_pair_to_pair_gap: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check(
        "differential-pair-to-pair-spacing-readiness",
        &[&board.source],
        || {
            let mut features = Vec::new();
            let mut pairs = Vec::new();
            let mut bounds = Vec::new();
            for feature in selected_copper_features(board, selected_layers) {
                if feature.kind == CopperKind::Via {
                    continue;
                }
                let Some(net) = &feature.net else {
                    continue;
                };
                let Some((pair, _side)) = differential_pair_key(net) else {
                    continue;
                };
                let Some(feature_bounds) = feature.region.try_to_multipolygon()?.bounding_rect()
                else {
                    continue;
                };
                features.push(feature);
                pairs.push(pair);
                bounds.push(feature_bounds);
            }
            let broad_phase_gap = scalar_broad_phase_radius(minimum_pair_to_pair_gap);
            let feature_index = CopperSpatialIndex::new(&features, broad_phase_gap);
            log::trace!(
                "differential pair-to-pair spacing readiness: source={} features={} buckets={} selected_layers={} threshold={:.6}",
                board.source,
                features.len(),
                feature_index.bucket_count(),
                selected_layers.len(),
                minimum_pair_to_pair_gap
            );

            let mut seen_pairs = BTreeSet::<(String, String, String)>::new();
            let mut candidate_pairs = 0_usize;
            let mut exact_pairs = 0_usize;
            let mut violations = Vec::new();

            for left_index in 0..features.len() {
                let left = features[left_index];
                for right_index in feature_index.same_layer_near_feature(left, broad_phase_gap) {
                    if right_index <= left_index {
                        continue;
                    }
                    candidate_pairs += 1;
                    if pairs[left_index] == pairs[right_index]
                        || !expanded_rects_overlap(
                            &bounds[left_index],
                            &bounds[right_index],
                            broad_phase_gap,
                        )
                    {
                        continue;
                    }
                    exact_pairs += 1;
                    let right = features[right_index];
                    let Some(gap) = crate::checks::boundary_distance_for_check(
                        &left.region,
                        &right.region,
                        "differential-pair-to-pair-spacing-readiness",
                        &[&left.layer],
                        &mut violations,
                    ) else {
                        continue;
                    };
                    if crate::checks::metric_cmp_for_check(
                        &gap,
                        minimum_pair_to_pair_gap,
                        "differential-pair-to-pair-spacing-readiness",
                        &[&left.layer],
                        &mut violations,
                    )
                    .is_none_or(|order| order.is_gt())
                    {
                        continue;
                    }

                    let (first_pair, second_pair) =
                        ordered_pair_names(&pairs[left_index], &pairs[right_index]);
                    if !seen_pairs.insert((
                        left.layer.clone(),
                        first_pair.clone(),
                        second_pair.clone(),
                    )) {
                        continue;
                    }

                    violations.push(Violation::new(
                "differential-pair-to-pair-spacing-readiness",
                Severity::Warning,
                vec![left.layer.clone()],
                None,
                Vec::new(),
                vec![
                    left.try_location_f64()?,
                    right.try_location_f64()?,
                ],
                Some(format!(
                    "likely differential pairs {first_pair} and {second_pair} have pair-to-pair copper spacing {gap:.6} on {}, below review threshold {minimum_pair_to_pair_gap:.6}; review crosstalk, impedance, and routing constraints",
                    left.layer
                )),
            ));
                }
            }

            log::trace!(
                "differential pair-to-pair spacing readiness: source={} candidate_pairs={} exact_pairs={} violations={}",
                board.source,
                candidate_pairs,
                exact_pairs,
                violations.len()
            );

            Ok(violations)
        },
    )
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum DifferentialSide {
    Positive,
    Negative,
}

#[derive(Default)]
struct PairLengthUse {
    positive: SideLengthUse,
    negative: SideLengthUse,
}

impl PairLengthUse {
    fn side_mut(&mut self, side: DifferentialSide) -> &mut SideLengthUse {
        match side {
            DifferentialSide::Positive => &mut self.positive,
            DifferentialSide::Negative => &mut self.negative,
        }
    }

    fn layers(&self) -> Vec<String> {
        self.positive
            .layers
            .union(&self.negative.layers)
            .cloned()
            .collect()
    }

    fn locations(&self) -> Vec<[f64; 2]> {
        let mut locations = self.positive.locations.clone();
        locations.extend(self.negative.locations.clone());
        locations
    }
}

#[derive(Default)]
struct PairWidthUse {
    positive: SideWidthUse,
    negative: SideWidthUse,
}

impl PairWidthUse {
    fn side_mut(&mut self, side: DifferentialSide) -> &mut SideWidthUse {
        match side {
            DifferentialSide::Positive => &mut self.positive,
            DifferentialSide::Negative => &mut self.negative,
        }
    }

    fn layers(&self) -> Vec<String> {
        self.positive
            .layers
            .union(&self.negative.layers)
            .cloned()
            .collect()
    }

    fn locations(&self) -> Vec<[f64; 2]> {
        let mut locations = self.positive.locations.clone();
        locations.extend(self.negative.locations.clone());
        locations
    }
}

#[derive(Default)]
struct SideWidthUse {
    layers: BTreeSet<String>,
    locations: Vec<[f64; 2]>,
    widths: Vec<Scalar>,
}

impl SideWidthUse {
    fn minimum_width(&self) -> Option<Scalar> {
        self.widths
            .iter()
            .filter(|width| crate::scalar::gt(width, &Scalar::zero()))
            .fold(None, |minimum, width| match minimum {
                Some(current) if crate::scalar::ge(width, &current) => Some(current),
                _ => Some(width.clone()),
            })
    }
}

struct SideLengthUse {
    layers: BTreeSet<String>,
    locations: Vec<[f64; 2]>,
    estimated_length: Scalar,
}

impl Default for SideLengthUse {
    fn default() -> Self {
        Self {
            layers: BTreeSet::new(),
            locations: Vec::new(),
            estimated_length: Scalar::zero(),
        }
    }
}

fn differential_pair_key(net: &str) -> Option<(String, DifferentialSide)> {
    let normalized = net.trim().to_ascii_uppercase();
    let patterns = [
        ("_DP", DifferentialSide::Positive),
        ("-DP", DifferentialSide::Positive),
        (".DP", DifferentialSide::Positive),
        ("_DM", DifferentialSide::Negative),
        ("-DM", DifferentialSide::Negative),
        (".DM", DifferentialSide::Negative),
        ("D+", DifferentialSide::Positive),
        ("D-", DifferentialSide::Negative),
        ("_P", DifferentialSide::Positive),
        ("-P", DifferentialSide::Positive),
        (".P", DifferentialSide::Positive),
        ("_N", DifferentialSide::Negative),
        ("-N", DifferentialSide::Negative),
        (".N", DifferentialSide::Negative),
    ];

    for (suffix, side) in patterns {
        let Some(base) = normalized.strip_suffix(suffix) else {
            continue;
        };
        let base = base
            .trim_end_matches(['_', '-', '.', '/', ' '])
            .trim_start_matches(['_', '-', '.', '/', ' ']);
        if !base.is_empty() {
            return Some((base.to_string(), side));
        }
    }

    None
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

fn looks_ground_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    normalized == "GND"
        || normalized == "GROUND"
        || normalized.starts_with("GND_")
        || normalized.starts_with("GND-")
        || normalized.contains("AGND")
        || normalized.contains("DGND")
        || normalized.contains("PGND")
        || normalized.contains("SHIELD")
        || normalized.contains("CHASSIS")
}

fn expanded_rects_overlap(left: &Rect<f64>, right: &Rect<f64>, expansion: f64) -> bool {
    left.min().x - expansion <= right.max().x
        && left.max().x + expansion >= right.min().x
        && left.min().y - expansion <= right.max().y
        && left.max().y + expansion >= right.min().y
}

fn ordered_pair_names(left: &str, right: &str) -> (String, String) {
    if left <= right {
        (left.to_string(), right.to_string())
    } else {
        (right.to_string(), left.to_string())
    }
}

fn estimated_segment_length(
    feature: &CopperFeature,
) -> Result<Option<Scalar>, crate::PcbGeometryUncertainty> {
    super::measurements::segment_length_proxy(&feature.region)
}

fn scalar_broad_phase_radius(value: &Scalar) -> f64 {
    let projected = value
        .to_f64_lossy()
        .expect("differential broad-phase radius must fit the finite compatibility index");
    if projected > 0.0 {
        projected.next_up()
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::{
        differential_pair_neckdown_readiness, differential_pair_skew_readiness,
        differential_pair_to_pair_spacing_readiness, differential_pair_via_proximity_readiness,
        differential_pair_via_return_readiness, differential_pair_width_readiness,
    };
    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, line_polygon, polygons_to_profile};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    #[test]
    fn differential_pair_neckdown_reports_long_narrow_segment() {
        let board = board_with_copper(vec![segment(
            "USB_DP",
            [0.0, 0.0],
            [3.0, 0.0],
            0.05,
            "F.Cu",
        )]);

        let violations = differential_pair_neckdown_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.08"),
            &crate::scalar::scalar("1.0"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "differential-pair-neckdown-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("neck-down"))
        );
    }

    #[test]
    fn differential_pair_neckdown_allows_wide_short_unpaired_or_selected_out_segments() {
        let wide = board_with_copper(vec![segment(
            "USB_DP",
            [0.0, 0.0],
            [3.0, 0.0],
            0.10,
            "F.Cu",
        )]);
        assert!(
            differential_pair_neckdown_readiness(
                &wide,
                &[],
                &crate::scalar::scalar("0.08"),
                &crate::scalar::scalar("1.0")
            )
            .is_empty()
        );

        let short = board_with_copper(vec![segment(
            "USB_DP",
            [0.0, 0.0],
            [0.5, 0.0],
            0.05,
            "F.Cu",
        )]);
        assert!(
            differential_pair_neckdown_readiness(
                &short,
                &[],
                &crate::scalar::scalar("0.08"),
                &crate::scalar::scalar("1.0")
            )
            .is_empty()
        );

        let unpaired =
            board_with_copper(vec![segment("GPIO", [0.0, 0.0], [3.0, 0.0], 0.05, "F.Cu")]);
        assert!(
            differential_pair_neckdown_readiness(
                &unpaired,
                &[],
                &crate::scalar::scalar("0.08"),
                &crate::scalar::scalar("1.0")
            )
            .is_empty()
        );

        let selected = vec!["B.Cu".to_string()];
        assert!(
            differential_pair_neckdown_readiness(
                &wide,
                &selected,
                &crate::scalar::scalar("0.08"),
                &crate::scalar::scalar("1.0")
            )
            .is_empty()
        );
    }

    #[test]
    fn differential_pair_neckdown_culls_large_wide_pair_sets() {
        let mut copper = Vec::new();
        for index in 0..800 {
            let y = index as f64 * 0.4;
            copper.push(segment(
                &format!("PAIR{index}_DP"),
                [0.0, y],
                [2.0, y],
                0.10,
                "F.Cu",
            ));
        }
        let board = board_with_copper(copper);

        let violations = differential_pair_neckdown_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.08"),
            &crate::scalar::scalar("1.0"),
        );

        assert!(
            violations.is_empty(),
            "wide inferred pair neckdown checks should stay linear in pair count"
        );
    }

    #[test]
    fn differential_pair_via_proximity_reports_unpaired_layer_change_vias() {
        let board = board_with_copper(vec![
            via("USB_DP", [0.0, 0.0], 0.30, "F.Cu"),
            via("USB_DM", [2.0, 0.0], 0.30, "F.Cu"),
        ]);

        let violations =
            differential_pair_via_proximity_readiness(&board, &[], &crate::scalar::scalar("0.50"));

        assert_eq!(violations.len(), 2);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "differential-pair-via-proximity-readiness")
        );
    }

    #[test]
    fn differential_pair_via_proximity_allows_nearby_missing_nonvia_or_selected_out_vias() {
        let nearby = board_with_copper(vec![
            via("USB_DP", [0.0, 0.0], 0.30, "F.Cu"),
            via("USB_DM", [0.2, 0.0], 0.30, "F.Cu"),
        ]);
        assert!(
            differential_pair_via_proximity_readiness(&nearby, &[], &crate::scalar::scalar("0.50"))
                .is_empty()
        );

        let missing = board_with_copper(vec![via("USB_DP", [0.0, 0.0], 0.30, "F.Cu")]);
        assert!(
            differential_pair_via_proximity_readiness(
                &missing,
                &[],
                &crate::scalar::scalar("0.50")
            )
            .is_empty()
        );

        let nonvia = board_with_copper(vec![
            segment("USB_DP", [0.0, 0.0], [1.0, 0.0], 0.10, "F.Cu"),
            via("USB_DM", [2.0, 0.0], 0.30, "F.Cu"),
        ]);
        assert!(
            differential_pair_via_proximity_readiness(&nonvia, &[], &crate::scalar::scalar("0.50"))
                .is_empty()
        );

        let selected = vec!["B.Cu".to_string()];
        assert!(
            differential_pair_via_proximity_readiness(
                &nearby,
                &selected,
                &crate::scalar::scalar("0.50")
            )
            .is_empty()
        );
    }

    #[test]
    fn differential_pair_via_proximity_culls_large_matched_via_sets() {
        let mut copper = Vec::new();
        for index in 0..600 {
            let x = index as f64 * 0.4;
            copper.push(via(&format!("PAIR{index}_DP"), [x, 0.0], 0.20, "F.Cu"));
            copper.push(via(
                &format!("PAIR{index}_DM"),
                [x + 0.1, 0.0],
                0.20,
                "F.Cu",
            ));
        }
        let board = board_with_copper(copper);

        let violations =
            differential_pair_via_proximity_readiness(&board, &[], &crate::scalar::scalar("0.20"));

        assert!(
            violations.is_empty(),
            "matched inferred pair via proximity checks should stay linear in pair count"
        );
    }

    #[test]
    fn differential_pair_via_proximity_culls_many_vias_within_one_pair() {
        let mut copper = Vec::new();
        for index in 0..1_000 {
            let x = index as f64 * 0.25;
            copper.push(via("DDR_DQS_DP", [x, 0.0], 0.20, "F.Cu"));
            copper.push(via("DDR_DQS_DM", [x + 0.05, 0.0], 0.20, "F.Cu"));
        }
        let board = board_with_copper(copper);
        let start = Instant::now();

        let violations =
            differential_pair_via_proximity_readiness(&board, &[], &crate::scalar::scalar("0.10"));

        assert!(violations.is_empty());
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "opposite-side via lookup should be indexed for dense pair escape fields"
        );
    }

    #[test]
    fn differential_pair_via_return_reports_missing_ground_stitching() {
        let board = board_with_copper(vec![
            via("USB_DP", [0.0, 0.0], 0.30, "F.Cu"),
            via("USB_DM", [0.2, 0.0], 0.30, "F.Cu"),
            via("GND", [2.0, 0.0], 0.30, "F.Cu"),
        ]);

        let violations =
            differential_pair_via_return_readiness(&board, &[], &crate::scalar::scalar("0.50"));

        assert_eq!(violations.len(), 2);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "differential-pair-via-return-readiness")
        );
    }

    #[test]
    fn differential_pair_via_return_allows_stitched_missing_unpaired_or_selected_out_vias() {
        let stitched = board_with_copper(vec![
            via("USB_DP", [0.0, 0.0], 0.30, "F.Cu"),
            via("USB_DM", [0.2, 0.0], 0.30, "F.Cu"),
            via("GND", [0.1, 0.0], 0.30, "F.Cu"),
        ]);
        assert!(
            differential_pair_via_return_readiness(&stitched, &[], &crate::scalar::scalar("0.50"))
                .is_empty()
        );

        let missing_side = board_with_copper(vec![
            via("USB_DP", [0.0, 0.0], 0.30, "F.Cu"),
            via("GND", [0.1, 0.0], 0.30, "F.Cu"),
        ]);
        assert!(
            differential_pair_via_return_readiness(
                &missing_side,
                &[],
                &crate::scalar::scalar("0.50")
            )
            .is_empty()
        );

        let selected = vec!["B.Cu".to_string()];
        assert!(
            differential_pair_via_return_readiness(
                &stitched,
                &selected,
                &crate::scalar::scalar("0.50")
            )
            .is_empty()
        );
    }

    #[test]
    fn differential_pair_via_return_culls_large_stitched_sets() {
        let mut copper = Vec::new();
        for index in 0..600 {
            let x = index as f64 * 0.4;
            copper.push(via(&format!("PAIR{index}_DP"), [x, 0.0], 0.20, "F.Cu"));
            copper.push(via(
                &format!("PAIR{index}_DM"),
                [x + 0.1, 0.0],
                0.20,
                "F.Cu",
            ));
            copper.push(via("GND", [x + 0.05, 0.0], 0.20, "F.Cu"));
        }
        let board = board_with_copper(copper);

        let violations =
            differential_pair_via_return_readiness(&board, &[], &crate::scalar::scalar("0.20"));

        assert!(
            violations.is_empty(),
            "stitched inferred pair via return checks should stay linear in pair count"
        );
    }

    #[test]
    fn differential_pair_via_return_culls_remote_ground_via_fields() {
        let mut copper = vec![
            via("USB_DP", [0.0, 0.0], 0.20, "F.Cu"),
            via("USB_DM", [0.05, 0.0], 0.20, "F.Cu"),
        ];
        for index in 0..2_000 {
            copper.push(via(
                "GND",
                [100.0 + index as f64 * 0.25, 10.0],
                0.20,
                "F.Cu",
            ));
        }
        let board = board_with_copper(copper);
        let start = Instant::now();

        let violations =
            differential_pair_via_return_readiness(&board, &[], &crate::scalar::scalar("0.20"));

        assert_eq!(violations.len(), 2);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "ground-stitch lookup should be indexed for sparse ground via fields"
        );
    }

    #[test]
    fn differential_pair_width_reports_narrow_pair_segment() {
        let board = board_with_copper(vec![
            segment("USB_DP", [0.0, 0.0], [2.0, 0.0], 0.05, "F.Cu"),
            segment("USB_DM", [0.0, 0.3], [2.0, 0.3], 0.10, "F.Cu"),
        ]);

        let violations = differential_pair_width_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.08"),
            &crate::scalar::scalar("0.05"),
        );

        // The adapter preserves these exact dyadic widths: 0.10_f64 -
        // 0.05_f64 is strictly greater than the decimal balance limit 0.05.
        // Rounding translated corners used to erase this second finding.
        assert_eq!(violations.len(), 2);
        assert_eq!(violations[0].check, "differential-pair-width-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("below review threshold"))
        );
        assert_eq!(violations[1].check, "differential-pair-width-readiness");
        assert!(
            violations[1]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("above balance threshold"))
        );
    }

    #[test]
    fn differential_pair_width_reports_pair_side_imbalance() {
        let board = board_with_copper(vec![
            segment("USB_DP", [0.0, 0.0], [2.0, 0.0], 0.20, "F.Cu"),
            segment("USB_DM", [0.0, 0.3], [2.0, 0.3], 0.10, "F.Cu"),
        ]);

        let violations = differential_pair_width_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.08"),
            &crate::scalar::scalar("0.04"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("side-width delta"))
        );
    }

    #[test]
    fn differential_pair_width_allows_balanced_missing_unsegmented_or_selected_out_pairs() {
        let balanced = board_with_copper(vec![
            segment("USB_DP", [0.0, 0.0], [2.0, 0.0], 0.10, "F.Cu"),
            segment("USB_DM", [0.0, 0.3], [2.0, 0.3], 0.11, "F.Cu"),
        ]);
        assert!(
            differential_pair_width_readiness(
                &balanced,
                &[],
                &crate::scalar::scalar("0.08"),
                &crate::scalar::scalar("0.04")
            )
            .is_empty()
        );

        let missing = board_with_copper(vec![segment(
            "USB_DP",
            [0.0, 0.0],
            [2.0, 0.0],
            0.10,
            "F.Cu",
        )]);
        assert!(
            differential_pair_width_readiness(
                &missing,
                &[],
                &crate::scalar::scalar("0.08"),
                &crate::scalar::scalar("0.04")
            )
            .is_empty()
        );

        let mut pad_feature = segment("USB_DP", [0.0, 0.0], [2.0, 0.0], 0.05, "F.Cu");
        pad_feature.kind = CopperKind::Pad;
        let unsegmented = board_with_copper(vec![pad_feature]);
        assert!(
            differential_pair_width_readiness(
                &unsegmented,
                &[],
                &crate::scalar::scalar("0.08"),
                &crate::scalar::scalar("0.04")
            )
            .is_empty()
        );

        let selected = vec!["B.Cu".to_string()];
        assert!(
            differential_pair_width_readiness(
                &balanced,
                &selected,
                &crate::scalar::scalar("0.08"),
                &crate::scalar::scalar("0.04")
            )
            .is_empty()
        );
    }

    #[test]
    fn differential_pair_width_culls_large_balanced_pair_sets() {
        let mut copper = Vec::new();
        for index in 0..600 {
            let y = index as f64 * 0.4;
            copper.push(segment(
                &format!("PAIR{index}_DP"),
                [0.0, y],
                [2.0, y],
                0.10,
                "F.Cu",
            ));
            copper.push(segment(
                &format!("PAIR{index}_DM"),
                [0.0, y + 0.2],
                [2.0, y + 0.2],
                0.10,
                "F.Cu",
            ));
        }
        let board = board_with_copper(copper);

        let violations = differential_pair_width_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.08"),
            &crate::scalar::scalar("0.02"),
        );

        assert!(
            violations.is_empty(),
            "balanced inferred pair width checks should stay linear in pair count"
        );
    }

    #[test]
    fn differential_pair_skew_reports_mismatched_segment_lengths() {
        let board = board_with_copper(vec![
            segment("USB_DP", [0.0, 0.0], [4.0, 0.0], 0.10, "F.Cu"),
            segment("USB_DM", [0.0, 0.3], [2.0, 0.3], 0.10, "F.Cu"),
        ]);

        let violations =
            differential_pair_skew_readiness(&board, &[], &crate::scalar::scalar("0.50"));

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "differential-pair-skew-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("USB") && message.contains("skew"))
        );
    }

    #[test]
    fn differential_pair_skew_allows_balanced_missing_unsegmented_or_selected_out_pairs() {
        let balanced = board_with_copper(vec![
            segment("USB_DP", [0.0, 0.0], [2.0, 0.0], 0.10, "F.Cu"),
            segment("USB_DM", [0.0, 0.3], [2.1, 0.3], 0.10, "F.Cu"),
        ]);
        assert!(
            differential_pair_skew_readiness(&balanced, &[], &crate::scalar::scalar("0.50"))
                .is_empty()
        );

        let missing = board_with_copper(vec![segment(
            "USB_DP",
            [0.0, 0.0],
            [4.0, 0.0],
            0.10,
            "F.Cu",
        )]);
        assert!(
            differential_pair_skew_readiness(&missing, &[], &crate::scalar::scalar("0.50"))
                .is_empty()
        );

        let selected = vec!["B.Cu".to_string()];
        assert!(
            differential_pair_skew_readiness(&balanced, &selected, &crate::scalar::scalar("0.50"))
                .is_empty()
        );
    }

    #[test]
    fn differential_pair_skew_culls_non_segment_geometry() {
        let mut pad_feature = segment("USB_DP", [0.0, 0.0], [4.0, 0.0], 0.10, "F.Cu");
        pad_feature.kind = CopperKind::Pad;
        let board = board_with_copper(vec![
            pad_feature,
            segment("USB_DM", [0.0, 0.3], [0.5, 0.3], 0.10, "F.Cu"),
        ]);

        assert!(
            differential_pair_skew_readiness(&board, &[], &crate::scalar::scalar("0.50"))
                .is_empty()
        );
    }

    #[test]
    fn differential_pair_skew_culls_large_balanced_pair_sets() {
        let mut copper = Vec::new();
        for index in 0..600 {
            let y = index as f64 * 0.4;
            copper.push(segment(
                &format!("PAIR{index}_DP"),
                [0.0, y],
                [2.0, y],
                0.10,
                "F.Cu",
            ));
            copper.push(segment(
                &format!("PAIR{index}_DM"),
                [0.0, y + 0.2],
                [2.0, y + 0.2],
                0.10,
                "F.Cu",
            ));
        }
        let board = board_with_copper(copper);

        let violations =
            differential_pair_skew_readiness(&board, &[], &crate::scalar::scalar("0.10"));

        assert!(
            violations.is_empty(),
            "balanced inferred pair skew checks should stay linear in pair count"
        );
    }

    #[test]
    fn differential_pair_to_pair_spacing_reports_crowded_pairs() {
        let board = board_with_copper(vec![
            segment("USB1_DP", [0.0, 0.00], [2.0, 0.00], 0.10, "F.Cu"),
            segment("USB1_DM", [0.0, 0.20], [2.0, 0.20], 0.10, "F.Cu"),
            segment("USB2_DP", [0.0, 0.42], [2.0, 0.42], 0.10, "F.Cu"),
            segment("USB2_DM", [0.0, 0.62], [2.0, 0.62], 0.10, "F.Cu"),
        ]);

        let violations = differential_pair_to_pair_spacing_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.30"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "differential-pair-to-pair-spacing-readiness"
        );
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("USB1") && message.contains("USB2"))
        );
    }

    #[test]
    fn differential_pair_to_pair_spacing_allows_same_pair_distant_unpaired_or_other_layer() {
        let board = board_with_copper(vec![
            segment("USB1_DP", [0.0, 0.00], [2.0, 0.00], 0.10, "F.Cu"),
            segment("USB1_DM", [0.0, 0.20], [2.0, 0.20], 0.10, "F.Cu"),
            segment("USB2_DP", [0.0, 2.00], [2.0, 2.00], 0.10, "F.Cu"),
            segment("USB2_DM", [0.0, 2.20], [2.0, 2.20], 0.10, "F.Cu"),
            segment("GPIO", [0.0, 0.35], [2.0, 0.35], 0.10, "F.Cu"),
            segment("USB3_DP", [0.0, 0.38], [2.0, 0.38], 0.10, "B.Cu"),
        ]);

        assert!(
            differential_pair_to_pair_spacing_readiness(
                &board,
                &[],
                &crate::scalar::scalar("0.30")
            )
            .is_empty()
        );
    }

    #[test]
    fn differential_pair_to_pair_spacing_respects_selected_layers() {
        let board = board_with_copper(vec![
            segment("USB1_DP", [0.0, 0.00], [2.0, 0.00], 0.10, "F.Cu"),
            segment("USB2_DP", [0.0, 0.20], [2.0, 0.20], 0.10, "F.Cu"),
        ]);

        let selected = vec!["B.Cu".to_string()];

        assert!(
            differential_pair_to_pair_spacing_readiness(
                &board,
                &selected,
                &crate::scalar::scalar("0.30")
            )
            .is_empty()
        );
    }

    #[test]
    fn differential_pair_to_pair_spacing_culls_large_sparse_pair_fields() {
        let mut copper = Vec::new();
        for index in 0..800 {
            let x = index as f64 * 2.0;
            copper.push(segment(
                &format!("PAIR{index}_DP"),
                [x, 0.0],
                [x + 0.5, 0.0],
                0.10,
                "F.Cu",
            ));
        }
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = differential_pair_to_pair_spacing_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.20"),
        );
        let elapsed = started.elapsed();

        assert!(
            violations.is_empty(),
            "pair-to-pair checks should cull distant pair features by bounds"
        );
        assert!(
            elapsed < std::time::Duration::from_secs(2),
            "sparse pair-to-pair field should stay in the spatial-index fast path, took {elapsed:?}"
        );
    }

    fn board_with_copper(copper: Vec<CopperFeature>) -> BoardModel {
        BoardModel {
            source: "test".to_string(),
            copper,
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        }
    }

    fn segment(
        net: &str,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
        layer: &str,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Segment,
            location: [
                crate::geometry::exact_real((start[0] + end[0]) / 2.0),
                crate::geometry::exact_real((start[1] + end[1]) / 2.0),
            ],
            region: polygons_to_profile(
                vec![line_polygon(start, end, width).expect("test segment should be valid")],
                Some(LayerMetadata {
                    name: layer.to_string(),
                }),
            ),
        }
    }

    fn via(net: &str, location: [f64; 2], diameter: f64, layer: &str) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Via,
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            region: polygons_to_profile(
                vec![circle_polygon(location, diameter / 2.0, 32)],
                Some(LayerMetadata {
                    name: layer.to_string(),
                }),
            ),
        }
    }
}
