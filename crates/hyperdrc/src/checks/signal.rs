//! Signal-integrity and mixed-signal partition readiness checks.
//!
//! These checks use KiCad net names and same-layer copper proximity to find
//! places where analog, RF, sensor, and other quiet nets need explicit
//! separation, guard, or return-path review.
//!
//! Reliability note: these checks do not solve fields or reconstruct complete
//! return-current paths. Results are suspect around split planes, unusual net
//! names, and intentional guard structures that are not parsed as copper.

use csgrs::csg::CSG;
use geo::BoundingRect;

use crate::checks::distance::polygon_boundary_distance;
use crate::checks::spatial::CopperSpatialIndex;
use crate::geometry::multipolygon_to_shapes;
use crate::kicad::{BoardModel, CopperFeature};
use crate::report::{Severity, Violation};

/// Warn when likely sensitive nets are too close to likely noisy nets.
///
/// This is a crosstalk and floor-planning heuristic, not a field solver. It
/// follows the practical mixed-signal layout guidance in Chesser and Porley,
/// "What Are the Basic Guidelines for Layout Design of Mixed-Signal PCBs?",
/// *Analog Dialogue* 56.3 (2022), which emphasizes physical separation of
/// analog and digital/noisy signals and a continuous low-impedance return path.
pub fn sensitive_net_spacing_readiness(
    board: &BoardModel,
    clearance: f64,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let sensitive_indices = features
        .iter()
        .enumerate()
        .filter_map(|(index, feature)| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_sensitive_net)
                .then_some(index)
        })
        .collect::<Vec<_>>();
    let noisy_features = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_noisy_net))
        .collect::<Vec<_>>();
    let noisy_index = CopperSpatialIndex::new(&noisy_features, clearance);
    log::trace!(
        "sensitive-net spacing readiness: source={} features={} sensitive={} noisy={} buckets={} clearance={clearance:.6}",
        board.source,
        features.len(),
        sensitive_indices.len(),
        noisy_features.len(),
        noisy_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;

    for &sensitive_index in &sensitive_indices {
        let sensitive = features[sensitive_index];
        for noisy_candidate_index in noisy_index.same_layer_near_feature(sensitive, clearance) {
            candidate_count += 1;
            let noisy = noisy_features[noisy_candidate_index];
            if std::ptr::eq(sensitive, noisy)
                || sensitive.net.is_none()
                || sensitive.net == noisy.net
            {
                continue;
            }
            if !sketches_within_clearance(&sensitive.sketch, &noisy.sketch, clearance) {
                continue;
            }

            let overlap = sensitive
                .sketch
                .offset(clearance)
                .intersection(&noisy.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let locations = if shapes.is_empty()
                && polygon_boundary_distance(
                    &sensitive.sketch.to_multipolygon(),
                    &noisy.sketch.to_multipolygon(),
                ) <= clearance
            {
                vec![sensitive.location, noisy.location]
            } else {
                Vec::new()
            };
            if shapes.is_empty() && locations.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "sensitive-net-spacing-readiness",
                Severity::Warning,
                vec![sensitive.layer.clone()],
                None,
                shapes,
                locations,
                Some(format!(
                    "likely sensitive net {:?} is within {clearance:.6} of likely noisy net {:?}; review analog/RF segregation and guard intent",
                    sensitive.net, noisy.net
                )),
            ));
        }
    }

    log::trace!(
        "sensitive-net spacing readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

/// Warn when sensitive analog/RF/sensor nets lack nearby same-layer ground.
///
/// The check looks for parsed ground copper inside `guard_distance`; it does
/// not prove impedance, shielding, or return-current density. It simply makes a
/// missing local guard or return feature visible in release review.
pub fn sensitive_return_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    guard_distance: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let ground_features = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .collect::<Vec<_>>();
    let ground_index = CopperSpatialIndex::new(&ground_features, guard_distance);
    log::trace!(
        "sensitive-return readiness: source={} features={} ground_features={} buckets={} guard_distance={guard_distance:.6}",
        board.source,
        features.len(),
        ground_features.len(),
        ground_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut sensitive_count = 0_usize;
    let mut candidate_count = 0_usize;

    for feature in features {
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_sensitive_net(net) {
            continue;
        }
        sensitive_count += 1;

        let guard_candidates = ground_index.same_layer_near_feature(feature, guard_distance);
        candidate_count += guard_candidates.len();
        let has_guard = guard_candidates.into_iter().any(|ground_index| {
            copper_features_touch(feature, ground_features[ground_index], guard_distance)
        });
        if has_guard {
            continue;
        }

        violations.push(Violation::new(
            "sensitive-return-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely sensitive net {net} has no parsed same-layer ground copper within {guard_distance:.6}; review guard, return, and shielding intent"
            )),
        ));
    }

    log::trace!(
        "sensitive-return readiness: source={} sensitive={} candidate_ground={} violations={}",
        board.source,
        sensitive_count,
        candidate_count,
        violations.len()
    );

    violations
}

/// Warn when analog/RF/sensor copper is close to digital/control copper
/// without nearby same-layer ground guarding the sensitive feature.
///
/// This extends `sensitive-net-spacing-readiness` to quieter digital nets such
/// as GPIO, SPI, I2C, UART, and MCU control lines that may not be classified as
/// high-speed or power-noisy. Chesser and Porley's mixed-signal layout article
/// describes this as physical floor planning: keep analog and digital signals
/// separated and provide a low-impedance ground reference instead of relying on
/// net names alone. Xu and Wang, "Investigating a guard trace ring to suppress
/// the crosstalk due to a clock trace on a power electronics DSP control board,"
/// *IEEE Transactions on Electromagnetic Compatibility* 57.3 (2015),
/// <https://doi.org/10.1109/TEMC.2015.2403289>, shows that grounded guard
/// geometry can materially change PCB crosstalk behavior.
pub fn mixed_signal_partition_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    separation: f64,
    guard_distance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let ground_features = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .collect::<Vec<_>>();
    let ground_index = CopperSpatialIndex::new(&ground_features, guard_distance);
    let sensitive_indices = features
        .iter()
        .enumerate()
        .filter_map(|(index, feature)| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_sensitive_net)
                .then_some(index)
        })
        .collect::<Vec<_>>();
    let digital_features = features
        .iter()
        .copied()
        .filter(|feature| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_digital_control_net)
        })
        .collect::<Vec<_>>();
    let digital_index = CopperSpatialIndex::new(&digital_features, separation);
    log::trace!(
        "mixed-signal partition readiness: source={} features={} sensitive={} digital={} digital_buckets={} ground_features={} ground_buckets={} separation={separation:.6} guard_distance={guard_distance:.6}",
        board.source,
        features.len(),
        sensitive_indices.len(),
        digital_features.len(),
        digital_index.bucket_count(),
        ground_features.len(),
        ground_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut digital_candidate_count = 0_usize;
    let mut guard_candidate_count = 0_usize;

    for &sensitive_index in &sensitive_indices {
        let sensitive = features[sensitive_index];
        for digital_candidate_index in digital_index.same_layer_near_feature(sensitive, separation)
        {
            digital_candidate_count += 1;
            let digital = digital_features[digital_candidate_index];
            if std::ptr::eq(sensitive, digital)
                || sensitive.net.is_none()
                || sensitive.net == digital.net
            {
                continue;
            }
            if !sketches_within_clearance(&sensitive.sketch, &digital.sketch, separation) {
                continue;
            }

            let guard_candidates = ground_index.same_layer_near_feature(sensitive, guard_distance);
            guard_candidate_count += guard_candidates.len();
            let has_guard = guard_candidates.into_iter().any(|ground_index| {
                copper_features_touch(sensitive, ground_features[ground_index], guard_distance)
            });
            if has_guard {
                continue;
            }

            let overlap = sensitive
                .sketch
                .offset(separation)
                .intersection(&digital.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let locations = if shapes.is_empty()
                && polygon_boundary_distance(
                    &sensitive.sketch.to_multipolygon(),
                    &digital.sketch.to_multipolygon(),
                ) <= separation
            {
                vec![sensitive.location, digital.location]
            } else {
                Vec::new()
            };
            if shapes.is_empty() && locations.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "mixed-signal-partition-readiness",
                Severity::Warning,
                vec![sensitive.layer.clone()],
                None,
                shapes,
                locations,
                Some(format!(
                    "likely sensitive net {:?} is within {separation:.6} of likely digital/control net {:?} without parsed same-layer ground guard within {guard_distance:.6}; review mixed-signal partitioning and return-current path",
                    sensitive.net, digital.net
                )),
            ));
        }
    }

    log::trace!(
        "mixed-signal partition readiness: source={} digital_candidates={} guard_candidates={} violations={}",
        board.source,
        digital_candidate_count,
        guard_candidate_count,
        violations.len()
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

fn looks_sensitive_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "RF", "ANT", "AUDIO", "MIC", "ADC", "DAC", "AIN", "AOUT", "ANALOG", "SENSE", "SNS", "XTAL",
        "CRYSTAL", "OSC",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_noisy_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = ["SW", "PHASE", "MOTOR", "PWM", "GATE", "BOOT", "DRIVE"];

    looks_high_current_net(net)
        || looks_high_speed_net(net)
        || tokens.iter().any(|token| normalized.contains(token))
}

fn looks_digital_control_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "GPIO", "MCU", "CPU", "FPGA", "SPI", "MOSI", "MISO", "SCK", "CS", "I2C", "SCL", "SDA",
        "UART", "TX", "RX", "JTAG", "SWD", "TCK", "TMS", "TDI", "TDO", "RESET", "IRQ", "INT",
        "ENABLE", "EN",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_high_current_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "VBUS", "VIN", "VCC", "VDD", "VOUT", "PWR", "POWER", "BATT", "BAT", "MOTOR", "LED",
        "HEATER", "LOAD",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_high_speed_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "USB", "D+", "D-", "DP", "DM", "CLK", "CLOCK", "TX", "RX", "SERDES", "PCIE", "PCI", "MIPI",
        "LVDS", "HDMI", "ETH", "RGMII", "SGMII", "SATA", "CAN",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_ground_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    matches!(
        normalized.as_str(),
        "GND" | "GROUND" | "PGND" | "AGND" | "DGND"
    ) || normalized.ends_with("_GND")
        || normalized.ends_with("-GND")
}

fn copper_features_touch(left: &CopperFeature, right: &CopperFeature, tolerance: f64) -> bool {
    if !sketches_within_clearance(&left.sketch, &right.sketch, tolerance) {
        return false;
    }

    if !left
        .sketch
        .intersection(&right.sketch)
        .to_multipolygon()
        .0
        .is_empty()
    {
        return true;
    }

    polygon_boundary_distance(
        &left.sketch.to_multipolygon(),
        &right.sketch.to_multipolygon(),
    ) <= tolerance
}

fn sketches_within_clearance(
    left: &crate::PcbSketch,
    right: &crate::PcbSketch,
    clearance: f64,
) -> bool {
    let Some(left_bounds) = left.geometry.bounding_rect() else {
        return true;
    };
    let Some(right_bounds) = right.geometry.bounding_rect() else {
        return true;
    };

    // Broad-phase bounding boxes conservatively reject only pairs that cannot
    // be within the clearance. Exact polygon offset/distance checks still
    // decide every candidate that survives, following the collision-detection
    // split in Lin and Canny, "A Fast Algorithm for Incremental Distance
    // Calculation", IEEE ICRA, 1991.
    left_bounds.min().x - clearance <= right_bounds.max().x
        && left_bounds.max().x + clearance >= right_bounds.min().x
        && left_bounds.min().y - clearance <= right_bounds.max().y
        && left_bounds.max().y + clearance >= right_bounds.min().y
}

#[cfg(test)]
mod tests {
    use crate::LayerMetadata;
    use crate::geometry::{line_polygon, polygons_to_sketch};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    use super::{
        mixed_signal_partition_readiness, sensitive_net_spacing_readiness,
        sensitive_return_readiness,
    };

    #[test]
    fn mixed_signal_partition_readiness_reports_unguarded_nearby_digital_control() {
        let board = board_with_copper(vec![
            copper_line("ADC_IN", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line(
                "MCU_GPIO1",
                CopperKind::Segment,
                [0.0, 0.25],
                [1.0, 0.25],
                0.10,
            ),
        ]);

        let violations = mixed_signal_partition_readiness(&board, &[], 0.30, 0.20, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "mixed-signal-partition-readiness");
    }

    #[test]
    fn mixed_signal_partition_readiness_allows_guarded_distant_or_selected_out() {
        let guarded = board_with_copper(vec![
            copper_line("ADC_IN", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line("AGND", CopperKind::Segment, [0.0, 0.16], [1.0, 0.16], 0.10),
            copper_line(
                "MCU_GPIO1",
                CopperKind::Segment,
                [0.0, 0.28],
                [1.0, 0.28],
                0.10,
            ),
        ]);
        let distant = board_with_copper(vec![
            copper_line("ADC_IN", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line(
                "MCU_GPIO1",
                CopperKind::Segment,
                [0.0, 1.0],
                [1.0, 1.0],
                0.10,
            ),
        ]);
        let selected_out = board_with_copper(vec![copper_line_on_layer(
            "ADC_IN",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        )]);

        assert!(mixed_signal_partition_readiness(&guarded, &[], 0.30, 0.20, 1.0e-9).is_empty());
        assert!(mixed_signal_partition_readiness(&distant, &[], 0.30, 0.20, 1.0e-9).is_empty());
        assert!(
            mixed_signal_partition_readiness(
                &selected_out,
                &["F.Cu".to_string()],
                0.30,
                0.20,
                1.0e-9
            )
            .is_empty()
        );
    }

    #[test]
    fn mixed_signal_partition_readiness_culls_sparse_digital_fields() {
        let mut copper = sparse_lines("MCU_GPIO", 2_000, 100.0);
        copper.push(copper_line(
            "ADC_IN",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        ));
        copper.push(copper_line(
            "MCU_GPIO_NEAR",
            CopperKind::Segment,
            [0.0, 0.25],
            [1.0, 0.25],
            0.10,
        ));
        copper.push(copper_line_on_layer(
            "MCU_GPIO_OTHER_LAYER",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.25],
            [1.0, 0.25],
            0.10,
        ));
        let board = board_with_copper(copper);

        let violations = mixed_signal_partition_readiness(&board, &[], 0.30, 0.20, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(violations[0].message.as_ref().is_some_and(|message| {
            message.contains("MCU_GPIO_NEAR") && !message.contains("MCU_GPIO_OTHER_LAYER")
        }));
    }

    #[test]
    fn sensitive_net_spacing_readiness_reports_sensitive_near_noisy_net() {
        let board = board_with_copper(vec![
            copper_line("RF_ANT", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
            copper_line(
                "MOTOR_PWM",
                CopperKind::Segment,
                [0.0, 0.35],
                [1.0, 0.35],
                0.1,
            ),
        ]);

        let violations = sensitive_net_spacing_readiness(&board, 0.30, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "sensitive-net-spacing-readiness");
    }

    #[test]
    fn sensitive_net_spacing_readiness_culls_sparse_noisy_fields() {
        let mut copper = sparse_lines("MOTOR_PWM", 2_000, 100.0);
        copper.push(copper_line(
            "RF_ANT",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        ));
        copper.push(copper_line(
            "MOTOR_PWM_NEAR",
            CopperKind::Segment,
            [0.0, 0.35],
            [1.0, 0.35],
            0.10,
        ));
        let board = board_with_copper(copper);

        let violations = sensitive_net_spacing_readiness(&board, 0.30, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "sensitive-net-spacing-readiness");
    }

    #[test]
    fn sensitive_return_readiness_reports_missing_same_layer_ground_guard() {
        let board = board_with_copper(vec![copper_line(
            "ADC_IN",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        )]);

        let violations = sensitive_return_readiness(&board, &[], 0.30);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "sensitive-return-readiness");
    }

    #[test]
    fn sensitive_return_readiness_culls_sparse_ground_fields() {
        let mut copper = sparse_lines("GND", 2_000, 100.0);
        copper.push(copper_line(
            "ADC_IN",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        ));
        copper.push(copper_line(
            "AGND",
            CopperKind::Segment,
            [0.0, 0.18],
            [1.0, 0.18],
            0.10,
        ));
        let board = board_with_copper(copper);

        assert!(sensitive_return_readiness(&board, &[], 0.30).is_empty());
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

    fn copper_line(
        net: &str,
        kind: CopperKind,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
    ) -> CopperFeature {
        copper_line_on_layer(net, kind, "F.Cu", start, end, width)
    }

    fn copper_line_on_layer(
        net: &str,
        kind: CopperKind,
        layer: &str,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind,
            location: [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0],
            sketch: polygons_to_sketch(
                vec![line_polygon(start, end, width).expect("test line should be valid")],
                Some(LayerMetadata {
                    name: "test line".to_string(),
                }),
            ),
        }
    }

    fn sparse_lines(prefix: &str, count: usize, offset_x: f64) -> Vec<CopperFeature> {
        (0..count)
            .map(|index| {
                let x = offset_x + (index % 100) as f64 * 3.0;
                let y = (index / 100) as f64 * 3.0;
                copper_line(
                    &format!("{prefix}{index}"),
                    CopperKind::Segment,
                    [x, y],
                    [x + 1.0, y],
                    0.10,
                )
            })
            .collect()
    }
}
