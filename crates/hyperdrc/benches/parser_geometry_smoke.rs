use std::time::Instant;

use hyperdrc::LayerMetadata;
use hyperdrc::baseline::report_to_waiver_stubs;
use hyperdrc::checks::{
    FileArtifact, TextArtifact, antenna_copper_keepout_readiness, apply_ipc356_nets,
    board_edge_exposure, board_outline_drill_clearance, castellation_pitch_readiness,
    chassis_stitching_readiness, component_edge_clearance_readiness,
    component_hole_clearance_readiness, component_spacing_readiness,
    conformal_coating_keepout_readiness, connector_return_path_readiness,
    connector_rework_clearance_readiness, controlled_impedance_readiness, copper_net_intent,
    copper_width_readiness, decoupling_proximity_readiness, dense_pad_escape_readiness,
    dense_pad_mask_bridge_readiness, dense_pad_via_spacing_readiness,
    different_net_short_readiness, differential_pair_neckdown_readiness,
    differential_pair_readiness, differential_pair_return_readiness,
    differential_pair_skew_readiness, differential_pair_spacing_readiness,
    differential_pair_to_pair_spacing_readiness, differential_pair_via_proximity_readiness,
    differential_pair_via_return_readiness, differential_pair_width_readiness, drill_spacing,
    drill_table_consistency, drill_to_copper_clearance, duplicate_layer_geometry_readiness,
    duplicate_layer_island_readiness, edge_plating_intent_readiness, edge_stitching_readiness,
    esd_protection_readiness, esd_return_path_readiness, exposed_copper,
    fiducial_keepout_readiness, fiducial_readiness, gold_finger_drill_keepout_readiness,
    gold_finger_edge_readiness, gold_finger_readiness, gold_finger_spacing_readiness,
    high_current_neck_readiness, high_current_readiness, high_speed_edge_readiness,
    high_voltage_edge_readiness, hot_component_spacing_readiness,
    inductor_copper_keepout_readiness, ipc356_coverage, ipc356_drill_diameter,
    local_copper_density_readiness, local_fiducial_readiness, mask_island_keepout,
    min_copper_neck_width, mixed_signal_partition_readiness,
    mounting_hole_copper_keepout_readiness, mounting_hole_distribution_readiness,
    mounting_hole_edge_clearance_readiness, mounting_hole_grounding_readiness,
    mounting_hole_plating_intent_readiness, mounting_hole_spacing_readiness, mouse_bite_readiness,
    net_constraint_readiness, net_spacing, orphaned_zone_readiness, pad_pair_asymmetry_readiness,
    panel_feature_outline_readiness, panelization_clearance, paste_aperture_coverage,
    paste_aperture_ratio, paste_aperture_spacing, paste_mask_alignment, paste_overhang,
    paste_via_exposure_readiness, plane_clearance_readiness, plating_intent,
    power_pad_entry_readiness, power_plane_readiness, power_via_array_readiness,
    power_via_return_readiness, press_fit_keepout_readiness, production_artifact_readiness,
    protective_earth_spacing_readiness, reference_plane_readiness, reference_plane_void_readiness,
    registration_tolerance, return_path_proximity_readiness, return_path_readiness,
    rf_keepout_readiness, rf_via_fence_readiness, same_net_drill_break_readiness,
    same_net_island_readiness, selective_wave_solder_keepout_readiness,
    sensitive_net_spacing_readiness, sensitive_return_readiness, silkscreen_clearance,
    silkscreen_overlap, skinny_layer_feature_readiness, solder_mask_expansion,
    solder_mask_opening_coverage, solder_mask_opening_spacing, solder_mask_overlap_clearance,
    split_plane_crossing_readiness, surge_protection_keepout_readiness,
    switch_node_keepout_readiness, teardrop_readiness, testpoint_accessibility_readiness,
    testpoint_copper_clearance_readiness, testpoint_coverage_readiness,
    thermal_copper_area_readiness, thermal_mechanical_keepout_readiness,
    thermal_pad_paste_windowpane_readiness, thermal_pad_via_readiness, thermal_relief_readiness,
    thermal_via_distribution_readiness, thermal_via_readiness, tiny_layer_feature_readiness,
    tombstone_paste_imbalance_readiness, tooling_hole_readiness,
    trace_junction_acid_trap_readiness, via_in_pad_readiness, voltage_clearance_readiness,
};
use hyperdrc::constraint_policy::{DifferentialRole, NetClassConfig};
use hyperdrc::geometry::{circle_polygon, line_polygon, polygons_to_sketch, rect_polygon};
use hyperdrc::ipc356::{Ipc356AccessSide, Ipc356FeatureType, Ipc356Point, Ipc356Soldermask};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use hyperdrc::report::{Report, Severity, Violation, report_summary};
use hyperdrc::sexp;
use hyperdrc::waiver::{Waiver, governance_violations};

fn main() {
    let sexp_input = r#"
        (kicad_pcb
          (net 1 "GND")
          (footprint "R" (at 10 20 0)
            (pad "1" smd rect (at 0 0) (size 1 2) (layers "F.Cu") (net 1 "GND"))))
    "#;

    let parse_elapsed = time("sexp_parse_10k", || {
        for _ in 0..10_000 {
            let _ = sexp::parse(sexp_input).expect("benchmark S-expression should parse");
        }
    });

    let geometry_elapsed = time("geometry_build_10k", || {
        for index in 0..10_000 {
            let x = index as f64 * 0.001;
            let _ = polygons_to_sketch(
                vec![
                    rect_polygon([x, x], [1.0, 2.0], 35.0),
                    circle_polygon([x + 2.0, x], 0.5, 32),
                ],
                Some(LayerMetadata {
                    name: "bench".to_string(),
                }),
            );
        }
    });
    let duplicate_layers = (0..16)
        .map(|index| {
            let x = (index % 4) as f64 * 12.0;
            let y = (index / 4) as f64 * 12.0;
            (
                format!("bench-layer-{index}"),
                polygons_to_sketch(
                    vec![rect_polygon([x, y], [8.0, 8.0], 0.0)],
                    Some(LayerMetadata {
                        name: format!("bench-layer-{index}"),
                    }),
                ),
            )
        })
        .collect::<Vec<_>>();
    let duplicate_layer_elapsed = time("duplicate_layer_geometry_1k", || {
        for _ in 0..1_000 {
            let _ = duplicate_layer_geometry_readiness(&duplicate_layers, 1.0e-9);
        }
    });
    let duplicate_island_layer = polygons_to_sketch(
        (0..100)
            .flat_map(|index| {
                let x = (index % 10) as f64 * 0.5;
                let y = (index / 10) as f64 * 0.5;
                let polygon = rect_polygon([x, y], [0.25, 0.25], 0.0);
                [polygon.clone(), polygon]
            })
            .collect(),
        Some(LayerMetadata {
            name: "bench duplicate islands".to_string(),
        }),
    );
    let duplicate_island_elapsed = time("duplicate_layer_island_1k", || {
        for _ in 0..1_000 {
            let _ = duplicate_layer_island_readiness(
                "bench duplicate islands",
                &duplicate_island_layer,
                1.0e-9,
            );
        }
    });
    let tiny_feature_layer = polygons_to_sketch(
        (0..100)
            .map(|index| {
                let x = (index % 10) as f64 * 0.2;
                let y = (index / 10) as f64 * 0.2;
                rect_polygon([x, y], [0.03, 0.03], 0.0)
            })
            .collect(),
        Some(LayerMetadata {
            name: "bench tiny features".to_string(),
        }),
    );
    let tiny_feature_elapsed = time("tiny_layer_feature_10k", || {
        for _ in 0..10_000 {
            let _ = tiny_layer_feature_readiness("bench tiny features", &tiny_feature_layer, 0.01);
        }
    });
    let skinny_feature_layer = polygons_to_sketch(
        (0..100)
            .map(|index| {
                let x = (index % 10) as f64 * 0.5;
                let y = (index / 10) as f64 * 0.2;
                rect_polygon([x, y], [0.35, 0.05], 0.0)
            })
            .collect(),
        Some(LayerMetadata {
            name: "bench skinny features".to_string(),
        }),
    );
    let skinny_feature_elapsed = time("skinny_layer_feature_10k", || {
        for _ in 0..10_000 {
            let _ = skinny_layer_feature_readiness(
                "bench skinny features",
                &skinny_feature_layer,
                0.10,
                0.01,
            );
        }
    });

    let density_layers = vec![
        (
            "F.Cu".to_string(),
            polygons_to_sketch(
                vec![rect_polygon([25.0, 25.0], [50.0, 50.0], 0.0)],
                Some(LayerMetadata {
                    name: "F.Cu".to_string(),
                }),
            ),
        ),
        (
            "B.Cu".to_string(),
            polygons_to_sketch(
                vec![rect_polygon([25.0, 25.0], [8.0, 8.0], 0.0)],
                Some(LayerMetadata {
                    name: "B.Cu".to_string(),
                }),
            ),
        ),
    ];
    let density_elapsed = time("local_copper_density_1k", || {
        for _ in 0..1_000 {
            let _ = local_copper_density_readiness(&density_layers, 10.0, 3.0, 1.0e-9);
        }
    });
    let sparse_copper_intent_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                let x = index as f64 * 2.0;
                bench_segment("SIG", [x, 0.0], [x + 1.0, 0.0], 0.16)
            })
            .chain([
                bench_segment("NARROW", [0.0, 2.0], [1.0, 2.0], 0.08),
                bench_unnetted_pad([2.0, 2.0], [0.30, 0.30]),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let copper_width_elapsed = time("copper_width_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = copper_width_readiness(&sparse_copper_intent_board, &[], 0.12);
        }
    });
    let copper_net_intent_elapsed = time("copper_net_intent_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = copper_net_intent(&sparse_copper_intent_board, &[]);
        }
    });
    let sparse_apertures = polygons_to_sketch(
        (0..2_000)
            .map(|index| rect_polygon([100.0 + index as f64 * 3.0, 10.0], [0.5, 0.5], 0.0))
            .chain([
                rect_polygon([0.5, 0.5], [1.0, 1.0], 0.0),
                rect_polygon([1.55, 0.5], [1.0, 1.0], 0.0),
            ])
            .collect(),
        Some(LayerMetadata {
            name: "bench sparse apertures".to_string(),
        }),
    );
    let paste_spacing_sparse_elapsed = time("paste_aperture_spacing_sparse_1k", || {
        for _ in 0..1_000 {
            let _ =
                paste_aperture_spacing("bench sparse apertures", &sparse_apertures, 0.10, 1.0e-9);
        }
    });
    let sparse_ratio_copper = polygons_to_sketch(
        vec![rect_polygon([0.5, 0.5], [1.0, 1.0], 0.0)],
        Some(LayerMetadata {
            name: "bench sparse ratio copper".to_string(),
        }),
    );
    let sparse_cover_copper = polygons_to_sketch(
        (0..2_000)
            .map(|index| rect_polygon([100.0 + index as f64 * 3.0, 10.0], [0.5, 0.5], 0.0))
            .chain([rect_polygon([0.4, 0.5], [0.8, 1.0], 0.0)])
            .collect(),
        Some(LayerMetadata {
            name: "bench sparse cover copper".to_string(),
        }),
    );
    let paste_ratio_sparse_elapsed = time("paste_aperture_ratio_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = paste_aperture_ratio(
                "bench sparse apertures",
                &sparse_apertures,
                "bench sparse ratio copper",
                &sparse_ratio_copper,
                0.50,
                1.20,
                1.0e-9,
            );
        }
    });
    let paste_coverage_sparse_elapsed = time("paste_aperture_coverage_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = paste_aperture_coverage(
                "bench sparse apertures",
                &sparse_apertures,
                "bench sparse ratio copper",
                &sparse_ratio_copper,
                1.0e-9,
            );
        }
    });
    let paste_overhang_sparse_elapsed = time("paste_overhang_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = paste_overhang(
                "bench sparse apertures",
                &sparse_ratio_copper,
                "bench sparse cover copper",
                &sparse_cover_copper,
                0.0,
                1.0e-9,
            );
        }
    });
    let mask_coverage_sparse_elapsed = time("solder_mask_opening_coverage_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = solder_mask_opening_coverage(
                "bench sparse ratio copper",
                &sparse_ratio_copper,
                "bench sparse apertures",
                &sparse_apertures,
                1.0e-9,
            );
        }
    });
    let exposed_copper_sparse_elapsed = time("exposed_copper_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = exposed_copper(
                "bench sparse ratio copper",
                &sparse_ratio_copper,
                "bench sparse apertures",
                &sparse_apertures,
                1.0e-9,
            );
        }
    });
    let mask_expansion_sparse_elapsed = time("solder_mask_expansion_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = solder_mask_expansion(
                "bench sparse cover copper",
                &sparse_cover_copper,
                "bench sparse apertures",
                &sparse_ratio_copper,
                0.10,
                1.0e-9,
            );
        }
    });
    let paste_mask_alignment_sparse_elapsed = time("paste_mask_alignment_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = paste_mask_alignment(
                "bench sparse apertures",
                &sparse_ratio_copper,
                "bench sparse apertures",
                &sparse_apertures,
                1.0e-9,
            );
        }
    });
    let mask_overlap_clearance_sparse_elapsed =
        time("solder_mask_overlap_clearance_sparse_1k", || {
            for _ in 0..1_000 {
                let _ = solder_mask_overlap_clearance(
                    "bench sparse ratio copper",
                    &sparse_ratio_copper,
                    "bench sparse apertures",
                    &sparse_apertures,
                    0.10,
                    1.0e-9,
                );
            }
        });
    let mask_spacing_sparse_elapsed = time("solder_mask_opening_spacing_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = solder_mask_opening_spacing(
                "bench sparse apertures",
                &sparse_apertures,
                0.10,
                1.0e-9,
            );
        }
    });
    let mask_island_sparse_elapsed = time("mask_island_keepout_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = mask_island_keepout("bench sparse apertures", &sparse_apertures, 0.10, 1.0e-9);
        }
    });
    let sparse_silk = polygons_to_sketch(
        vec![
            line_polygon([-0.2, 0.5], [1.2, 0.5], 0.08)
                .expect("benchmark silkscreen line should be valid"),
        ],
        Some(LayerMetadata {
            name: "bench sparse silk".to_string(),
        }),
    );
    let silkscreen_overlap_sparse_elapsed = time("silkscreen_overlap_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = silkscreen_overlap(
                "bench sparse silk",
                &sparse_silk,
                "bench sparse apertures",
                &sparse_apertures,
                1.0e-9,
            );
        }
    });
    let silkscreen_clearance_sparse_elapsed = time("silkscreen_clearance_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = silkscreen_clearance(
                "bench sparse silk",
                &sparse_silk,
                "bench sparse apertures",
                &sparse_apertures,
                0.10,
                1.0e-9,
            );
        }
    });
    let tombstone_copper = polygons_to_sketch(
        (0..1_000)
            .map(|index| {
                let x = 100.0 + index as f64 * 5.0;
                rect_polygon([x + 0.5, 0.5], [1.0, 1.0], 0.0)
            })
            .chain([
                rect_polygon([0.5, 0.5], [1.0, 1.0], 0.0),
                rect_polygon([1.9, 0.5], [1.0, 1.0], 0.0),
            ])
            .collect(),
        Some(LayerMetadata {
            name: "bench tombstone copper".to_string(),
        }),
    );
    let tombstone_paste = polygons_to_sketch(
        vec![
            rect_polygon([0.5, 0.5], [1.0, 1.0], 0.0),
            rect_polygon([1.65, 0.5], [0.5, 1.0], 0.0),
        ],
        Some(LayerMetadata {
            name: "bench tombstone paste".to_string(),
        }),
    );
    let tombstone_elapsed = time("tombstone_paste_imbalance_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = tombstone_paste_imbalance_readiness(
                "bench tombstone paste",
                &tombstone_paste,
                "bench tombstone copper",
                &tombstone_copper,
                2.0,
                0.30,
                1.0e-9,
            );
        }
    });
    let paste_via_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![bench_via("GND", [0.0, 0.0], 0.20)],
        drills: vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.20,
            net: Some("GND".to_string()),
            plated: true,
        }],
        board_outline: None,
        panel_features: None,
    };
    let paste_via_sparse_paste = polygons_to_sketch(
        (0..2_000)
            .map(|index| {
                let x = 100.0 + index as f64 * 5.0;
                rect_polygon([x + 0.5, 0.5], [1.0, 1.0], 0.0)
            })
            .chain([rect_polygon([0.0, 0.0], [0.4, 0.4], 0.0)])
            .collect(),
        Some(LayerMetadata {
            name: "bench paste via sparse paste".to_string(),
        }),
    );
    let thermal_pad_windowpane_copper = polygons_to_sketch(
        vec![rect_polygon([0.0, 0.0], [4.0, 4.0], 0.0)],
        Some(LayerMetadata {
            name: "bench thermal copper".to_string(),
        }),
    );
    let paste_via_elapsed = time("paste_via_exposure_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = paste_via_exposure_readiness(
                "bench paste via sparse paste",
                &paste_via_sparse_paste,
                &paste_via_sparse_board,
                &[],
                1.0e-9,
            );
        }
    });
    let thermal_pad_windowpane_sparse_elapsed =
        time("thermal_pad_paste_windowpane_sparse_1k", || {
            for _ in 0..1_000 {
                let _ = thermal_pad_paste_windowpane_readiness(
                    "bench paste via sparse paste",
                    &paste_via_sparse_paste,
                    "bench thermal copper",
                    &thermal_pad_windowpane_copper,
                    4.0,
                    0.65,
                    1.0e-9,
                );
            }
        });
    let net_constraint_classes = vec![NetClassConfig {
        name: "bench-power".to_string(),
        nets: vec!["VBUS".to_string()],
        min_clearance: Some(0.4),
        ..NetClassConfig::default()
    }];
    let mut net_constraint_copper = (0..1_000)
        .map(|index| {
            bench_pad(
                &format!("SIG{index}"),
                [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                [0.2, 0.2],
            )
        })
        .collect::<Vec<_>>();
    net_constraint_copper.push(bench_pad("VBUS", [0.0, 0.0], [0.2, 0.2]));
    net_constraint_copper.push(bench_pad("SIG_NEAR", [0.3, 0.0], [0.2, 0.2]));
    let net_constraint_board = BoardModel {
        source: "bench".to_string(),
        copper: net_constraint_copper,
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let net_constraint_elapsed = time("net_constraint_clearance_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = net_constraint_readiness(
                &net_constraint_classes,
                None,
                std::slice::from_ref(&net_constraint_board),
                &[],
            );
        }
    });
    let net_constraint_pair_classes = vec![
        NetClassConfig {
            name: "bench-pair-p".to_string(),
            nets: vec!["USB_D+".to_string()],
            differential_pair: Some("usb".to_string()),
            differential_role: Some(DifferentialRole::Positive),
            min_pair_spacing: Some(0.2),
            max_pair_spacing: Some(0.5),
            ..NetClassConfig::default()
        },
        NetClassConfig {
            name: "bench-pair-n".to_string(),
            nets: vec!["USB_D-".to_string()],
            differential_pair: Some("usb".to_string()),
            differential_role: Some(DifferentialRole::Negative),
            min_pair_spacing: Some(0.2),
            max_pair_spacing: Some(0.5),
            ..NetClassConfig::default()
        },
    ];
    let mut net_constraint_pair_copper = (0..500)
        .flat_map(|index| {
            [
                bench_pad("USB_D+", [100.0 + index as f64 * 4.0, 0.0], [0.10, 0.10]),
                bench_pad("USB_D-", [100.0 + index as f64 * 4.0, 2.0], [0.10, 0.10]),
            ]
        })
        .collect::<Vec<_>>();
    net_constraint_pair_copper.push(bench_pad("USB_D+", [0.0, 0.0], [0.10, 0.10]));
    net_constraint_pair_copper.push(bench_pad("USB_D-", [0.18, 0.0], [0.10, 0.10]));
    let net_constraint_pair_board = BoardModel {
        source: "bench".to_string(),
        copper: net_constraint_pair_copper,
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let net_constraint_pair_elapsed = time("net_constraint_pair_spacing_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = net_constraint_readiness(
                &net_constraint_pair_classes,
                None,
                std::slice::from_ref(&net_constraint_pair_board),
                &[],
            );
        }
    });
    let different_net_spacing_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_pad(
                    &format!("SIG{index}"),
                    [100.0 + index as f64 * 2.0, 100.0],
                    [0.10, 0.10],
                )
            })
            .chain([
                bench_pad("A", [0.0, 0.0], [0.20, 0.20]),
                bench_pad("B", [0.25, 0.0], [0.20, 0.20]),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let different_net_spacing_elapsed = time("different_net_spacing_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = net_spacing(&different_net_spacing_board, 0.10, &[], 1.0e-9);
        }
    });
    let registration_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .flat_map(|index| {
                let location = [100.0 + index as f64 * 2.0, 100.0];
                [
                    bench_pad_on_layer("F.Cu", &format!("F{index}"), location, [0.10, 0.10]),
                    bench_pad_on_layer(
                        "B.Cu",
                        &format!("B{index}"),
                        [location[0] + 0.8, location[1] + 0.8],
                        [0.10, 0.10],
                    ),
                ]
            })
            .chain([
                bench_pad_on_layer("F.Cu", "F_NEAR", [0.0, 0.0], [0.20, 0.20]),
                bench_pad_on_layer("B.Cu", "B_NEAR", [0.25, 0.0], [0.20, 0.20]),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let registration_elapsed = time("registration_tolerance_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = registration_tolerance(&registration_board, 0.10, 1.0e-9);
        }
    });

    let acid_trap_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_segment("SIG", [0.0, 0.0], [2.0, 0.0], 0.12),
            bench_segment("SIG", [0.0, 0.0], [1.9, 0.7], 0.12),
            bench_segment("GND", [4.0, 0.0], [6.0, 0.0], 0.12),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let acid_trap_elapsed = time("trace_junction_acid_trap_10k", || {
        for _ in 0..10_000 {
            let _ = trace_junction_acid_trap_readiness(&acid_trap_board, &[], 30.0, 1.0e-9);
        }
    });
    let acid_trap_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                let y = index as f64 * 0.50;
                bench_segment("SIG", [100.0, y], [101.0, y], 0.10)
            })
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let acid_trap_sparse_elapsed = time("trace_junction_acid_trap_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = trace_junction_acid_trap_readiness(&acid_trap_sparse_board, &[], 30.0, 1.0e-9);
        }
    });

    let via_in_pad_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    &format!("GND_{index}"),
                    [100.0 + index as f64 * 4.0, 100.0],
                    [0.6, 0.6],
                )
            })
            .chain([
                bench_pad("GND", [0.0, 0.0], [0.6, 0.6]),
                bench_via("GND", [0.05, 0.0], 0.24),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let via_in_pad_elapsed = time("via_in_pad_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = via_in_pad_readiness(&via_in_pad_board, &[], 1.0e-9);
        }
    });

    let teardrop_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    &format!("SIG_{index}"),
                    [100.0 + index as f64 * 4.0, 100.0],
                    [0.5, 0.5],
                )
            })
            .chain([
                bench_pad("SIG", [0.0, 0.0], [0.5, 0.5]),
                bench_segment("SIG", [0.0, 0.0], [1.0, 0.0], 0.08),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let teardrop_elapsed = time("teardrop_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = teardrop_readiness(&teardrop_board, &[], 0.12, 1.0e-9);
        }
    });

    let dense_pad_board = BoardModel {
        source: "bench".to_string(),
        copper: bench_dense_pad_cluster(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let dense_pad_fiducial_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| bench_fiducial([100.0 + index as f64 * 4.0, 100.0], 0.8))
            .chain(bench_dense_pad_cluster().into_iter())
            .chain([
                bench_fiducial([-1.0, -1.0], 0.8),
                bench_fiducial([2.5, -1.0], 0.8),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let local_fiducial_elapsed = time("local_fiducial_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = local_fiducial_readiness(&dense_pad_fiducial_board, &[], 0.8, 5.0);
        }
    });
    let dense_pad_escape_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| bench_via("ESC", [100.0 + index as f64 * 4.0, 100.0], 0.20))
            .chain(bench_dense_pad_cluster().into_iter())
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let dense_pad_escape_elapsed = time("dense_pad_escape_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = dense_pad_escape_readiness(&dense_pad_escape_board, &[], 0.8, 2.0);
        }
    });
    let dense_pad_via_elapsed = time("dense_pad_via_spacing_5k", || {
        for _ in 0..5_000 {
            let _ = dense_pad_via_spacing_readiness(&dense_pad_board, &[], 0.8, 2.0, 0.15, 1.0e-9);
        }
    });
    let dense_pad_via_sparse_pads_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_pad(
                    &format!("P{index}"),
                    [(index % 50) as f64 * 0.5, (index / 50) as f64 * 0.5],
                    [0.20, 0.20],
                )
            })
            .chain([bench_via("ESC_NEAR", [0.32, 0.0], 0.20)])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let dense_pad_via_sparse_pads_elapsed = time("dense_pad_via_spacing_sparse_pads_1k", || {
        for _ in 0..1_000 {
            let _ = dense_pad_via_spacing_readiness(
                &dense_pad_via_sparse_pads_board,
                &[],
                0.8,
                25.0,
                0.15,
                1.0e-9,
            );
        }
    });

    let dense_pad_mask_elapsed = time("dense_pad_mask_bridge_10k", || {
        for _ in 0..10_000 {
            let _ = dense_pad_mask_bridge_readiness(&dense_pad_board, &[], 0.8, 0.10);
        }
    });
    let dense_pad_mask_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_pad(
                    &format!("P{index}"),
                    [
                        100.0 + (index % 50) as f64 * 1.0,
                        100.0 + (index / 50) as f64 * 1.0,
                    ],
                    [0.25, 0.25],
                )
            })
            .chain(bench_dense_pad_cluster_with_size(0.45))
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let dense_pad_mask_sparse_elapsed = time("dense_pad_mask_bridge_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = dense_pad_mask_bridge_readiness(&dense_pad_mask_sparse_board, &[], 0.8, 0.10);
        }
    });

    let assembly_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                bench_pad(
                    &format!("J{index}"),
                    [(index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    [1.0, 0.8],
                )
            })
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let component_spacing_elapsed = time("component_spacing_1k", || {
        for _ in 0..1_000 {
            let _ = component_spacing_readiness(&assembly_sparse_board, &[], 0.25, 0.5);
        }
    });
    let component_edge_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_pad(
                    &format!("U{index}_IO"),
                    [
                        20.0 + (index % 80) as f64 * 2.0,
                        20.0 + (index / 80) as f64 * 2.0,
                    ],
                    [0.3, 0.3],
                )
            })
            .chain([bench_pad("U_NEAR", [0.65, 100.0], [0.3, 0.3])])
            .collect(),
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([100.0, 100.0], [200.0, 200.0], 0.0)],
            Some(LayerMetadata {
                name: "bench component edge outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let component_edge_elapsed = time("component_edge_clearance_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = component_edge_clearance_readiness(&component_edge_board, &[], 0.5);
        }
    });
    let component_hole_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                bench_pad(
                    &format!("U{index}"),
                    [(index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    [0.5, 0.5],
                )
            })
            .collect(),
        drills: (0..100)
            .map(|index| {
                bench_drill(
                    [500.0 + (index % 20) as f64 * 4.0, (index / 20) as f64 * 4.0],
                    0.5,
                    false,
                )
            })
            .collect(),
        board_outline: None,
        panel_features: None,
    };
    let component_hole_elapsed = time("component_hole_clearance_1k", || {
        for _ in 0..1_000 {
            let _ =
                component_hole_clearance_readiness(&component_hole_board, &[], &[], 0.25, 1.0e-9);
        }
    });
    let connector_rework_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                let net = if index % 80 == 0 {
                    format!("USB_CONN_{index}")
                } else {
                    format!("SIG{index}")
                };
                bench_pad(
                    &net,
                    [(index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    [1.0, 0.8],
                )
            })
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let connector_rework_elapsed = time("connector_rework_clearance_1k", || {
        for _ in 0..1_000 {
            let _ = connector_rework_clearance_readiness(&connector_rework_board, &[], 0.25, 0.5);
        }
    });
    let connector_return_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| bench_via("GND", [100.0 + index as f64 * 2.0, 50.0], 0.12))
            .chain([bench_pad("USB_D_P", [0.7, 8.3], [0.6, 0.6])])
            .collect(),
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([10.0, 10.0], [20.0, 20.0], 0.0)],
            Some(LayerMetadata {
                name: "bench connector outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let connector_return_sparse_elapsed = time("connector_return_path_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = connector_return_path_readiness(&connector_return_sparse_board, &[], 1.0, 2.0);
        }
    });
    let edge_stitching_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| bench_via("GND", [100.0 + index as f64 * 2.0, 50.0], 0.12))
            .chain([bench_segment("USB_D_P", [0.1, 1.0], [0.9, 1.0], 0.10)])
            .collect(),
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
            Some(LayerMetadata {
                name: "bench edge-stitch outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let edge_stitching_sparse_elapsed = time("edge_stitching_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = edge_stitching_readiness(&edge_stitching_sparse_board, &[], 0.50, 0.30, 1.0e-9);
        }
    });
    let rectangular_edge_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_pad(
                    &format!("SIG{index}"),
                    [
                        5.0 + (index % 50) as f64 * 1.5,
                        5.0 + (index / 50) as f64 * 1.5,
                    ],
                    [0.30, 0.30],
                )
            })
            .chain([bench_pad("EDGE", [99.95, 50.0], [0.30, 0.30])])
            .collect(),
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([50.0, 50.0], [100.0, 100.0], 0.0)],
            Some(LayerMetadata {
                name: "bench rectangular edge outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let board_edge_exposure_elapsed = time("board_edge_exposure_rect_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = board_edge_exposure(&rectangular_edge_board, &[], 1.0e-9);
        }
    });
    let rectangular_high_speed_edge_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                let y = 5.0 + (index / 50) as f64 * 1.5;
                let x = 5.0 + (index % 50) as f64 * 1.5;
                bench_segment(&format!("USB_D{index}_P"), [x, y], [x + 0.5, y], 0.10)
            })
            .chain([bench_segment("PCIE_RX0", [0.10, 50.0], [0.90, 50.0], 0.10)])
            .collect(),
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([50.0, 50.0], [100.0, 100.0], 0.0)],
            Some(LayerMetadata {
                name: "bench rectangular high-speed edge outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let high_speed_edge_elapsed = time("high_speed_edge_rect_sparse_1k", || {
        for _ in 0..1_000 {
            let _ =
                high_speed_edge_readiness(&rectangular_high_speed_edge_board, &[], 0.50, 1.0e-9);
        }
    });
    let rectangular_high_voltage_edge_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                let y = 5.0 + (index / 50) as f64 * 1.5;
                let x = 5.0 + (index % 50) as f64 * 1.5;
                bench_segment(&format!("HV_BUS_{index}"), [x, y], [x + 0.5, y], 0.10)
            })
            .chain([bench_segment("MAINS_L", [0.20, 50.0], [1.0, 50.0], 0.10)])
            .collect(),
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([50.0, 50.0], [100.0, 100.0], 0.0)],
            Some(LayerMetadata {
                name: "bench rectangular high-voltage edge outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let high_voltage_edge_elapsed = time("high_voltage_edge_rect_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = high_voltage_edge_readiness(
                &rectangular_high_voltage_edge_board,
                &[],
                0.80,
                1.0e-9,
            );
        }
    });
    let chassis_stitching_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| bench_via("GND", [100.0 + index as f64 * 2.0, 50.0], 0.12))
            .chain([bench_segment("USB_SHIELD", [0.0, 0.0], [1.0, 0.0], 0.20)])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let chassis_stitching_sparse_elapsed = time("chassis_stitching_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = chassis_stitching_readiness(&chassis_stitching_sparse_board, &[], 0.50);
        }
    });
    let asymmetry_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                let size = if index % 2 == 0 {
                    [0.5, 0.5]
                } else {
                    [0.55, 0.5]
                };
                bench_pad(
                    &format!("R{index}"),
                    [(index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    size,
                )
            })
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let pad_pair_asymmetry_elapsed = time("pad_pair_asymmetry_1k", || {
        for _ in 0..1_000 {
            let _ = pad_pair_asymmetry_readiness(&asymmetry_board, &[], 0.30, 1.5, 2.0);
        }
    });
    let mut fiducial_copper = (0..400)
        .map(|index| {
            bench_pad(
                &format!("SIG{index}"),
                [(index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                [0.25, 0.25],
            )
        })
        .collect::<Vec<_>>();
    fiducial_copper.push(bench_fiducial([-10.0, -10.0], 0.8));
    let fiducial_keepout_board = BoardModel {
        source: "bench".to_string(),
        copper: fiducial_copper,
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let fiducial_keepout_elapsed = time("fiducial_keepout_1k", || {
        for _ in 0..1_000 {
            let _ = fiducial_keepout_readiness(&fiducial_keepout_board, &[], 0.25, 1.0e-9);
        }
    });
    let fiducial_edge_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_fiducial(
                    [
                        20.0 + (index % 80) as f64 * 2.0,
                        20.0 + (index / 80) as f64 * 2.0,
                    ],
                    0.5,
                )
            })
            .collect(),
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([100.0, 100.0], [200.0, 200.0], 0.0)],
            Some(LayerMetadata {
                name: "bench fiducial outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let fiducial_edge_elapsed = time("fiducial_edge_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = fiducial_readiness(&fiducial_edge_board, &[], 1.0);
        }
    });
    let process_keepout_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                bench_pad(
                    &format!("SIG{index}"),
                    [(index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    [0.25, 0.25],
                )
            })
            .collect(),
        drills: (0..100)
            .map(|index| {
                bench_net_drill(
                    if index % 2 == 0 {
                        "HEADER_PIN"
                    } else {
                        "PRESS_FIT_CONN"
                    },
                    [500.0 + (index % 20) as f64 * 4.0, (index / 20) as f64 * 4.0],
                    0.5,
                    true,
                )
            })
            .collect(),
        board_outline: None,
        panel_features: None,
    };
    let selective_wave_elapsed = time("selective_wave_keepout_1k", || {
        for _ in 0..1_000 {
            let _ =
                selective_wave_solder_keepout_readiness(&process_keepout_board, &[], 0.25, 1.0e-9);
        }
    });
    let press_fit_elapsed = time("press_fit_keepout_1k", || {
        for _ in 0..1_000 {
            let _ = press_fit_keepout_readiness(&process_keepout_board, &[], 0.35, 1.0e-9);
        }
    });
    let mouse_bite_drills = (0..1_000)
        .flat_map(|index| {
            let x = index as f64 * 10.0;
            [
                DrillFeature {
                    location: [x, 0.0],
                    diameter: 0.30,
                    net: None,
                    plated: false,
                },
                DrillFeature {
                    location: [x + 0.70, 0.0],
                    diameter: 0.30,
                    net: None,
                    plated: false,
                },
            ]
        })
        .chain([DrillFeature {
            location: [50_000.0, 0.0],
            diameter: 0.30,
            net: None,
            plated: false,
        }])
        .collect::<Vec<_>>();
    let mouse_bite_board = BoardModel {
        source: "bench".to_string(),
        copper: Vec::new(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let mouse_bite_elapsed = time("mouse_bite_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = mouse_bite_readiness(
                &mouse_bite_board,
                &mouse_bite_drills,
                0.25,
                0.50,
                0.40,
                1.20,
            );
        }
    });
    let tooling_hole_board = BoardModel {
        source: "bench".to_string(),
        copper: Vec::new(),
        drills: (0..2_000)
            .map(|index| DrillFeature {
                location: [200.0 + index as f64 * 2.0, 200.0],
                diameter: 0.40,
                net: None,
                plated: false,
            })
            .chain([
                DrillFeature {
                    location: [10.0, 10.0],
                    diameter: 1.50,
                    net: None,
                    plated: false,
                },
                DrillFeature {
                    location: [90.0, 90.0],
                    diameter: 1.50,
                    net: None,
                    plated: false,
                },
            ])
            .collect(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([50.0, 50.0], [100.0, 100.0], 0.0)],
            Some(LayerMetadata {
                name: "bench tooling outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let tooling_hole_elapsed = time("tooling_hole_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = tooling_hole_readiness(&tooling_hole_board, &[], 0.8, 4.0, 1.0);
        }
    });
    let conformal_coating_board = BoardModel {
        source: "bench".to_string(),
        copper: std::iter::once(bench_pad("USB_DP", [0.0, 0.0], [0.4, 0.4]))
            .chain((0..2_000).map(|index| {
                bench_pad(
                    &format!("SIG{index}"),
                    [
                        10.0 + (index % 50) as f64 * 3.0,
                        10.0 + (index / 50) as f64 * 3.0,
                    ],
                    [0.3, 0.3],
                )
            }))
            .chain([bench_pad("SIG_NEAR", [0.55, 0.0], [0.3, 0.3])])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let conformal_coating_elapsed = time("conformal_coating_keepout_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = conformal_coating_keepout_readiness(&conformal_coating_board, &[], 0.3, 1.0e-9);
        }
    });

    let testpoints = (0..400)
        .map(|index| {
            bench_testpoint(
                &format!("TP{index}"),
                [(index % 40) as f64 * 1.5, (index / 40) as f64 * 1.5],
                0.45,
            )
        })
        .collect::<Vec<_>>();
    let testpoint_board = BoardModel {
        source: "bench".to_string(),
        copper: Vec::new(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let testpoint_access_elapsed = time("testpoint_accessibility_1k", || {
        for _ in 0..1_000 {
            let _ =
                testpoint_accessibility_readiness(&testpoint_board, &testpoints, 0.40, 0.25, 1.0);
        }
    });
    let coverage_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_pad(
                    &format!("VBUS_{index}"),
                    [index as f64 * 2.0, 0.0],
                    [0.3, 0.3],
                )
            })
            .chain([bench_pad("VDD_MISSING", [9_000.0, 0.0], [0.3, 0.3])])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let coverage_points = (0..2_000)
        .map(|index| bench_testpoint(&format!("VBUS_{index}"), [index as f64 * 2.0, 0.0], 0.4))
        .collect::<Vec<_>>();
    let testpoint_coverage_elapsed = time("testpoint_coverage_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = testpoint_coverage_readiness(&coverage_board, &coverage_points, &[]);
        }
    });
    let testpoint_side_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    "TP_SIDE",
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    [0.25, 0.25],
                )
            })
            .chain([bench_pad_on_layer(
                "B.Cu",
                "TP_SIDE",
                [-10.0, -10.0],
                [0.35, 0.35],
            )])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let testpoint_side_points = vec![bench_testpoint("TP_SIDE", [-10.0, -10.0], 0.40)];
    let testpoint_side_elapsed = time("testpoint_side_parity_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = testpoint_accessibility_readiness(
                &testpoint_side_board,
                &testpoint_side_points,
                0.40,
                0.35,
                1.0,
            );
        }
    });
    let testpoint_copper_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    &format!("SIG{index}"),
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    [0.25, 0.25],
                )
            })
            .chain([
                bench_pad("TP_NET", [-10.0, -10.0], [0.40, 0.40]),
                bench_pad("OTHER_NEAR", [-9.62, -10.0], [0.25, 0.25]),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let testpoint_copper_points = vec![bench_testpoint("TP_NET", [-10.0, -10.0], 0.40)];
    let testpoint_copper_elapsed = time("testpoint_copper_clearance_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = testpoint_copper_clearance_readiness(
                &testpoint_copper_board,
                &testpoint_copper_points,
                &[],
                0.40,
                0.10,
                1.0e-9,
            );
        }
    });

    let antenna_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_segment("WIFI_ANT", [0.0, 0.0], [2.0, 0.0], 0.12),
            bench_segment("GND", [0.0, 0.45], [2.0, 0.45], 0.12),
            bench_segment("GPIO", [4.0, 0.0], [5.0, 0.0], 0.12),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let antenna_keepout_elapsed = time("antenna_copper_keepout_10k", || {
        for _ in 0..10_000 {
            let _ = antenna_copper_keepout_readiness(&antenna_board, &[], 0.60, 1.0e-9);
        }
    });
    let rf_keepout_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                bench_segment(
                    &format!("GPIO{index}"),
                    [100.0 + (index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    [101.0 + (index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    0.12,
                )
            })
            .chain([
                bench_segment("RF_FEED", [0.0, 0.0], [2.0, 0.0], 0.12),
                bench_segment("GPIO_NEAR", [0.0, 0.45], [2.0, 0.45], 0.12),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let rf_keepout_elapsed = time("rf_keepout_1k", || {
        for _ in 0..1_000 {
            let _ = rf_keepout_readiness(&rf_keepout_board, 0.60, &[], 1.0e-9);
        }
    });
    let rf_fence_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                bench_via(
                    "GND",
                    [100.0 + (index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    0.20,
                )
            })
            .chain([
                bench_segment("RF_FEED", [0.0, 0.0], [2.0, 0.0], 0.12),
                bench_via("GND", [0.2, 0.2], 0.20),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let rf_via_fence_elapsed = time("rf_via_fence_1k", || {
        for _ in 0..1_000 {
            let _ = rf_via_fence_readiness(&rf_fence_board, &[], 0.60);
        }
    });

    let power_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_segment("BUCK_LX", [0.0, 0.0], [2.0, 0.0], 0.20),
            bench_segment("PGND", [0.0, 0.55], [2.0, 0.55], 0.20),
            bench_segment("ADC_IN", [4.0, 0.0], [5.0, 0.0], 0.12),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let inductor_keepout_elapsed = time("inductor_copper_keepout_10k", || {
        for _ in 0..10_000 {
            let _ = inductor_copper_keepout_readiness(&power_board, &[], 0.70, 1.0e-9);
        }
    });
    let switch_node_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                bench_segment(
                    &format!("GPIO{index}"),
                    [100.0 + (index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    [101.0 + (index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    0.12,
                )
            })
            .chain([
                bench_segment("BUCK_SW", [0.0, 0.0], [2.0, 0.0], 0.20),
                bench_segment("ADC_NEAR", [0.0, 0.45], [2.0, 0.45], 0.12),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let switch_node_elapsed = time("switch_node_keepout_1k", || {
        for _ in 0..1_000 {
            let _ = switch_node_keepout_readiness(&switch_node_board, &[], 0.60, 1.0e-9);
        }
    });
    let sparse_power_summary_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                let x = index as f64 * 2.0;
                bench_segment("GPIO", [x, 0.0], [x + 1.0, 0.0], 0.12)
            })
            .chain([bench_segment("VBUS", [0.0, 2.0], [1.0, 2.0], 0.18)])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let power_plane_elapsed = time("power_plane_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = power_plane_readiness(&sparse_power_summary_board, &[]);
        }
    });
    let high_current_neck_elapsed = time("high_current_neck_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = high_current_neck_readiness(&sparse_power_summary_board, &[], 0.30);
        }
    });
    let pad_entry_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_pad("VIN", [0.0, 0.0], [1.0, 1.0]),
            bench_segment("VIN", [0.5, 0.0], [2.0, 0.0], 0.12),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let power_pad_entry_elapsed = time("power_pad_entry_10k", || {
        for _ in 0..10_000 {
            let _ = power_pad_entry_readiness(&pad_entry_board, &[], 0.20, 0.30, 2);
        }
    });
    let sparse_pad_entry_board = BoardModel {
        source: "bench".to_string(),
        copper: std::iter::once(bench_pad("VIN", [0.0, 0.0], [1.0, 1.0]))
            .chain((0..2_000).map(|index| {
                bench_segment(
                    "VIN",
                    [
                        100.0 + (index % 100) as f64 * 2.0,
                        100.0 + (index / 100) as f64 * 2.0,
                    ],
                    [
                        101.0 + (index % 100) as f64 * 2.0,
                        100.0 + (index / 100) as f64 * 2.0,
                    ],
                    0.50,
                )
            }))
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let power_pad_entry_sparse_elapsed = time("power_pad_entry_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = power_pad_entry_readiness(&sparse_pad_entry_board, &[], 0.20, 0.30, 2);
        }
    });
    let power_via_return_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_via("VIN", [0.0, 0.0], 0.20),
            bench_segment("GND", [2.0, 0.0], [3.0, 0.0], 0.20),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let power_via_return_elapsed = time("power_via_return_10k", || {
        for _ in 0..10_000 {
            let _ = power_via_return_readiness(&power_via_return_board, &[], 0.50);
        }
    });
    let sparse_power_via_return_board = BoardModel {
        source: "bench".to_string(),
        copper: std::iter::once(bench_via("VIN", [0.0, 0.0], 0.20))
            .chain((0..2_000).map(|index| {
                bench_segment(
                    "GND",
                    [
                        100.0 + (index % 100) as f64 * 2.0,
                        100.0 + (index / 100) as f64 * 2.0,
                    ],
                    [
                        101.0 + (index % 100) as f64 * 2.0,
                        100.0 + (index / 100) as f64 * 2.0,
                    ],
                    0.20,
                )
            }))
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let power_via_return_sparse_elapsed = time("power_via_return_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = power_via_return_readiness(&sparse_power_via_return_board, &[], 0.50);
        }
    });
    let power_via_array_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| bench_via("VBUS", [index as f64 * 2.0, 0.0], 0.20))
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let power_via_array_sparse_elapsed = time("power_via_array_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = power_via_array_readiness(&power_via_array_board, &[], 0.50);
        }
    });
    let decoupling_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| bench_pad("GND", [100.0 + index as f64 * 2.0, 20.0], [0.3, 0.3]))
            .chain([bench_pad("VDD_3V3", [0.0, 0.0], [0.3, 0.3])])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let decoupling_sparse_elapsed = time("decoupling_proximity_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = decoupling_proximity_readiness(&decoupling_sparse_board, &[], 1.0);
        }
    });

    let thermal_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_zone("VOUT", [0.0, 0.0], [3.0, 2.0]),
            bench_via("VOUT", [0.0, 0.0], 0.20),
            bench_via("VOUT", [0.25, 0.0], 0.20),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let thermal_via_distribution_elapsed = time("thermal_via_distribution_10k", || {
        for _ in 0..10_000 {
            let _ = thermal_via_distribution_readiness(&thermal_board, &[], 2, 1.0, 0.10);
        }
    });
    let thermal_via_cluster_board = BoardModel {
        source: "bench".to_string(),
        copper: std::iter::once(bench_zone("VOUT", [0.5, 0.4], [4.0, 4.0]))
            .chain((0..1_000).map(|index| {
                bench_via(
                    "VOUT",
                    [(index % 50) as f64 * 0.02, (index / 50) as f64 * 0.02],
                    0.01,
                )
            }))
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let thermal_via_cluster_elapsed = time("thermal_via_distribution_clustered_100x1k", || {
        for _ in 0..100 {
            let _ =
                thermal_via_distribution_readiness(&thermal_via_cluster_board, &[], 2, 5.0, 0.0);
        }
    });
    let thermal_via_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: std::iter::once(bench_zone("VOUT", [0.0, 0.0], [2.0, 2.0]))
            .chain((0..1_000).map(|index| {
                bench_via(
                    "VOUT",
                    [
                        100.0 + (index % 50) as f64 * 3.0,
                        100.0 + (index / 50) as f64 * 3.0,
                    ],
                    0.02,
                )
            }))
            .chain([
                bench_via("VOUT", [0.0, 0.0], 0.20),
                bench_via("VOUT", [0.25, 0.0], 0.20),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let thermal_via_elapsed = time("thermal_via_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = thermal_via_readiness(&thermal_via_sparse_board, &[], 3, 0.10);
        }
    });
    let thermal_via_distribution_sparse_elapsed =
        time("thermal_via_distribution_sparse_1k", || {
            for _ in 0..1_000 {
                let _ = thermal_via_distribution_readiness(
                    &thermal_via_sparse_board,
                    &[],
                    2,
                    1.0,
                    0.10,
                );
            }
        });
    let thermal_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                bench_zone(
                    &format!("SIG{index}"),
                    [100.0 + (index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    [0.5, 0.5],
                )
            })
            .chain([
                bench_pad("VOUT", [0.0, 0.0], [0.5, 0.5]),
                bench_zone("VOUT", [1.0, 0.0], [1.0, 1.0]),
                bench_pad("LED_PWR", [0.0, 2.0], [1.0, 1.0]),
                bench_pad("SENSOR_NEAR", [1.2, 2.0], [0.8, 1.0]),
            ])
            .collect(),
        drills: vec![bench_drill([1.4, 2.0], 0.8, false)],
        board_outline: None,
        panel_features: None,
    };
    let thermal_copper_area_elapsed = time("thermal_copper_area_1k", || {
        for _ in 0..1_000 {
            let _ = thermal_copper_area_readiness(&thermal_sparse_board, &[], 2.0);
        }
    });
    let thermal_relief_elapsed = time("thermal_relief_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = thermal_relief_readiness(&thermal_sparse_board, &[], 1.0e-9);
        }
    });
    let thermal_pad_via_elapsed = time("thermal_pad_via_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = thermal_pad_via_readiness(&thermal_sparse_board, &[], 0.75);
        }
    });
    let hot_component_spacing_elapsed = time("hot_component_spacing_1k", || {
        for _ in 0..1_000 {
            let _ = hot_component_spacing_readiness(&thermal_sparse_board, &[], 0.3, 1.0e-9);
        }
    });
    let thermal_mechanical_elapsed = time("thermal_mechanical_keepout_1k", || {
        for _ in 0..1_000 {
            let _ =
                thermal_mechanical_keepout_readiness(&thermal_sparse_board, &[], &[], 0.2, 1.0e-9);
        }
    });

    let safety_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_pad("USB_TVS_CLAMP", [0.0, 0.0], [0.35, 0.35]),
            bench_pad("CHASSIS", [0.28, 0.0], [0.40, 0.40]),
            bench_segment("USB_D_P", [1.0, 0.0], [2.0, 0.0], 0.12),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let esd_return_path_elapsed = time("esd_return_path_10k", || {
        for _ in 0..10_000 {
            let _ = esd_return_path_readiness(&safety_board, &[], 0.50);
        }
    });
    let esd_protection_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_pad("USB_D_P", [0.6, 5.0], [0.4, 0.4]),
            bench_pad("USB_TVS_CLAMP", [1.6, 5.0], [0.4, 0.4]),
        ],
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([10.0, 10.0], [20.0, 20.0], 0.0)],
            Some(LayerMetadata {
                name: "bench outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let esd_protection_elapsed = time("esd_protection_1k", || {
        for _ in 0..1_000 {
            let _ = esd_protection_readiness(&esd_protection_board, &[], 1.0, 2.0);
        }
    });
    let protective_spacing_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_segment("HV_BUS", [0.0, 0.0], [1.0, 0.0], 0.20),
            bench_segment("PE", [1.3, 0.0], [2.3, 0.0], 0.20),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let protective_earth_spacing_elapsed = time("protective_earth_spacing_10k", || {
        for _ in 0..10_000 {
            let _ =
                protective_earth_spacing_readiness(&protective_spacing_board, &[], 0.30, 1.0e-9);
        }
    });
    let voltage_clearance_elapsed = time("voltage_clearance_1k", || {
        for _ in 0..1_000 {
            let _ = voltage_clearance_readiness(&protective_spacing_board, 0.30, &[], 1.0e-9);
        }
    });
    let surge_keepout_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_pad("MOV_LINE", [0.0, 0.0], [0.5, 0.5]),
            bench_segment("GPIO", [0.8, 0.0], [1.6, 0.0], 0.20),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let surge_keepout_elapsed = time("surge_protection_keepout_10k", || {
        for _ in 0..10_000 {
            let _ = surge_protection_keepout_readiness(&surge_keepout_board, &[], 0.30, 1.0e-9);
        }
    });

    let signal_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_segment("ADC_IN", [0.0, 0.0], [2.0, 0.0], 0.10),
            bench_segment("MCU_GPIO1", [0.0, 0.35], [2.0, 0.35], 0.10),
            bench_segment("AGND", [0.0, 1.00], [2.0, 1.00], 0.12),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let mixed_signal_partition_elapsed = time("mixed_signal_partition_10k", || {
        for _ in 0..10_000 {
            let _ = mixed_signal_partition_readiness(&signal_board, &[], 0.45, 0.20, 1.0e-9);
        }
    });
    let sparse_signal_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                bench_segment(
                    &format!("MOTOR_PWM{index}"),
                    [100.0 + (index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    [101.0 + (index % 40) as f64 * 4.0, (index / 40) as f64 * 4.0],
                    0.12,
                )
            })
            .chain([
                bench_segment("ADC_IN", [0.0, 0.0], [2.0, 0.0], 0.10),
                bench_segment("MOTOR_PWM_NEAR", [0.0, 0.35], [2.0, 0.35], 0.10),
                bench_segment("AGND", [0.0, 0.80], [2.0, 0.80], 0.12),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let sensitive_spacing_elapsed = time("sensitive_net_spacing_1k", || {
        for _ in 0..1_000 {
            let _ = sensitive_net_spacing_readiness(&sparse_signal_board, 0.45, &[], 1.0e-9);
        }
    });
    let sensitive_return_elapsed = time("sensitive_return_1k", || {
        for _ in 0..1_000 {
            let _ = sensitive_return_readiness(&sparse_signal_board, &[], 0.30);
        }
    });

    let neck_layer = polygons_to_sketch(
        (0..120)
            .map(|index| {
                let x = (index % 20) as f64 * 0.4;
                let y = (index / 20) as f64 * 0.4;
                rect_polygon([x, y], [0.20, 0.08], 0.0)
            })
            .collect(),
        Some(LayerMetadata {
            name: "bench neck islands".to_string(),
        }),
    );
    let min_copper_neck_elapsed = time("min_copper_neck_1k", || {
        for _ in 0..1_000 {
            let _ = min_copper_neck_width("bench neck islands", &neck_layer, 0.0762, 1.0e-9);
        }
    });

    let continuity_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![bench_segment("SIG", [-1.0, 0.0], [1.0, 0.0], 0.30)],
        drills: vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.60,
            net: None,
            plated: false,
        }],
        board_outline: None,
        panel_features: None,
    };
    let continuity_elapsed = time("same_net_drill_break_10k", || {
        for _ in 0..10_000 {
            let _ = same_net_drill_break_readiness(&continuity_board, &[], &[], 1.0e-9);
        }
    });
    let continuity_sparse_drill_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![bench_segment("SIG", [-1.0, 0.0], [1.0, 0.0], 0.30)],
        drills: (0..2_000)
            .map(|index| DrillFeature {
                location: [10.0 + index as f64 * 2.0, 10.0],
                diameter: 0.60,
                net: None,
                plated: false,
            })
            .chain(std::iter::once(DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.60,
                net: None,
                plated: false,
            }))
            .collect(),
        board_outline: None,
        panel_features: None,
    };
    let continuity_sparse_drills_elapsed = time("same_net_drill_break_sparse_drills_1k", || {
        for _ in 0..1_000 {
            let _ =
                same_net_drill_break_readiness(&continuity_sparse_drill_board, &[], &[], 1.0e-9);
        }
    });
    let same_net_island_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                let x = index as f64 * 2.0;
                bench_segment("GPIO1", [x, 0.0], [x + 0.5, 0.0], 0.10)
            })
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let same_net_island_sparse_elapsed = time("same_net_island_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = same_net_island_readiness(&same_net_island_sparse_board, &[], 0.10);
        }
    });
    let plane_clearance_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                let x = 100.0 + index as f64 * 3.0;
                bench_zone("GND", [x + 0.5, 100.5], [1.0, 1.0])
            })
            .collect(),
        drills: (0..400)
            .map(|index| DrillFeature {
                location: [index as f64 * 3.0, 0.0],
                diameter: 0.5,
                net: None,
                plated: false,
            })
            .collect(),
        board_outline: None,
        panel_features: None,
    };
    let plane_clearance_sparse_elapsed = time("plane_clearance_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = plane_clearance_readiness(&plane_clearance_sparse_board, &[], 1.0e-9);
        }
    });
    let panelization_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_pad(
                    &format!("SIG{index}"),
                    [
                        100.0 + (index % 50) as f64 * 5.0,
                        100.0 + (index / 50) as f64 * 5.0,
                    ],
                    [0.08, 0.08],
                )
            })
            .chain([bench_pad("NEAR_TAB", [0.12, 0.0], [0.08, 0.08])])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: Some(polygons_to_sketch(
            vec![
                line_polygon([0.0, -1.0], [0.0, 1.0], 0.05)
                    .expect("benchmark panel route line should be valid"),
            ],
            Some(LayerMetadata {
                name: "bench sparse panel features".to_string(),
            }),
        )),
    };
    let panelization_clearance_elapsed = time("panelization_clearance_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = panelization_clearance(&panelization_sparse_board, &[], 0.25, 1.0e-9);
        }
    });

    let drill_clearance_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..400)
            .map(|index| {
                bench_pad(
                    &format!("N{index}"),
                    [20.0 + index as f64 * 2.0, 20.0],
                    [0.4, 0.4],
                )
            })
            .chain(std::iter::once(bench_segment(
                "SIG",
                [-1.0, 0.0],
                [1.0, 0.0],
                0.20,
            )))
            .collect(),
        drills: vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.40,
            net: None,
            plated: false,
        }],
        board_outline: None,
        panel_features: None,
    };
    let drill_clearance_elapsed = time("drill_to_copper_clearance_sparse_1k", || {
        for _ in 0..1_000 {
            let _ =
                drill_to_copper_clearance(&drill_clearance_sparse_board, &[], 0.20, &[], 1.0e-9);
        }
    });
    let plating_intent_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_pad(
                    &format!("SIG{index}"),
                    [100.0 + index as f64 * 2.0, 100.0],
                    [0.4, 0.4],
                )
            })
            .chain([
                bench_pad("GND", [0.01, 0.0], [0.4, 0.4]),
                bench_pad("SIG_NEAR", [0.0, 2.0], [0.4, 0.4]),
            ])
            .collect(),
        drills: vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.30,
                net: Some("GND".to_string()),
                plated: true,
            },
            DrillFeature {
                location: [0.0, 2.0],
                diameter: 0.60,
                net: None,
                plated: false,
            },
        ],
        board_outline: None,
        panel_features: None,
    };
    let plating_intent_elapsed = time("plating_intent_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = plating_intent(&plating_intent_board, &[], 0.05);
        }
    });
    let drill_outline = polygons_to_sketch(
        vec![rect_polygon([50.0, 50.0], [100.0, 100.0], 0.0)],
        Some(LayerMetadata {
            name: "bench outline".to_string(),
        }),
    );
    let mut outline_clearance_drills = (0..2_000)
        .map(|index| {
            bench_drill(
                [
                    5.0 + (index % 50) as f64 * 1.5,
                    5.0 + (index / 50) as f64 * 1.5,
                ],
                0.30,
                false,
            )
        })
        .collect::<Vec<_>>();
    outline_clearance_drills.push(bench_drill([0.35, 50.0], 0.30, false));
    let board_outline_drill_elapsed = time("board_outline_drill_clearance_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = board_outline_drill_clearance(
                "KiCad drills",
                "KiCad Edge.Cuts",
                &drill_outline,
                &outline_clearance_drills,
                &[],
                0.25,
                1.0e-9,
            );
        }
    });

    let sparse_drills = (0..1_000)
        .map(|index| bench_drill([20.0 + index as f64 * 2.0, 20.0], 0.30, true))
        .chain([
            bench_drill([0.0, 0.0], 0.40, true),
            bench_drill([0.55, 0.0], 0.40, true),
        ])
        .collect::<Vec<_>>();
    let drill_spacing_elapsed = time("drill_spacing_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = drill_spacing(&sparse_drills, &[], 0.20);
        }
    });

    let board_table_drills = vec![bench_drill([0.0, 0.0], 0.40, true)];
    let mut sidecar_table_drills = (0..1_000)
        .map(|index| bench_drill([20.0 + index as f64 * 2.0, 20.0], 0.60, true))
        .collect::<Vec<_>>();
    sidecar_table_drills.push(bench_drill([0.05, 0.0], 0.60, true));
    let mut ipc_table_points = (0..1_000)
        .map(|index| bench_testpoint("SIG", [40.0 + index as f64 * 2.0, 40.0], 0.80))
        .collect::<Vec<_>>();
    ipc_table_points.push(bench_testpoint("SIG", [0.06, 0.0], 0.80));
    let drill_table_elapsed = time("drill_table_consistency_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = drill_table_consistency(
                &board_table_drills,
                &sidecar_table_drills,
                &ipc_table_points,
                0.10,
            );
        }
    });
    let ipc356_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    "SIG",
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    [0.25, 0.25],
                )
            })
            .chain([bench_pad("IPC_NET", [0.0, 0.0], [0.35, 0.35])])
            .collect(),
        drills: (0..1_000)
            .map(|index| bench_drill([100.0 + index as f64 * 5.0, 20.0], 0.30, true))
            .chain([bench_drill([0.0, 1.0], 0.30, true)])
            .collect(),
        board_outline: None,
        panel_features: None,
    };
    let ipc356_points = vec![
        bench_testpoint("IPC_NET", [0.02, 0.0], 0.35),
        bench_testpoint("IPC_DRILL", [0.02, 1.0], 0.50),
    ];
    let ipc356_apply_elapsed = time("ipc356_apply_sparse_1k", || {
        for _ in 0..1_000 {
            let mut board = ipc356_board.clone();
            apply_ipc356_nets(&mut board, &ipc356_points, 0.05);
        }
    });
    let ipc356_coverage_elapsed = time("ipc356_coverage_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = ipc356_coverage(&ipc356_board, &ipc356_points, 0.05);
        }
    });
    let ipc356_drill_elapsed = time("ipc356_drill_diameter_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = ipc356_drill_diameter(&ipc356_board, &ipc356_points, 0.05);
        }
    });

    let mechanical_spacing_board = BoardModel {
        source: "bench".to_string(),
        copper: Vec::new(),
        drills: (0..1_000)
            .map(|index| bench_drill([20.0 + index as f64 * 4.0, 20.0], 3.0, false))
            .chain([
                bench_drill([0.0, 0.0], 3.0, false),
                bench_drill([3.4, 0.0], 3.0, true),
            ])
            .collect(),
        board_outline: None,
        panel_features: None,
    };
    let mounting_hole_spacing_elapsed = time("mounting_hole_spacing_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = mounting_hole_spacing_readiness(&mechanical_spacing_board, 0.5);
        }
    });
    let mounting_hole_distribution_board = BoardModel {
        source: "bench".to_string(),
        copper: Vec::new(),
        drills: (0..1_000)
            .map(|index| {
                bench_drill(
                    [
                        10.0 + (index % 50) as f64 * 0.01,
                        10.0 + (index / 50) as f64 * 0.01,
                    ],
                    3.2,
                    false,
                )
            })
            .collect(),
        board_outline: None,
        panel_features: None,
    };
    let mounting_hole_distribution_elapsed = time("mounting_hole_distribution_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = mounting_hole_distribution_readiness(&mounting_hole_distribution_board, 8.0);
        }
    });
    let mounting_hole_grounding_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    "GND",
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    [0.25, 0.25],
                )
            })
            .collect(),
        drills: vec![bench_drill([-10.0, -10.0], 3.2, false)],
        board_outline: None,
        panel_features: None,
    };
    let mounting_hole_grounding_elapsed = time("mounting_hole_grounding_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = mounting_hole_grounding_readiness(&mounting_hole_grounding_board, &[], 1.0);
        }
    });
    let mounting_hole_keepout_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    &format!("SIG{index}"),
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    [0.25, 0.25],
                )
            })
            .chain([bench_pad("SIG_NEAR", [-8.8, -10.0], [0.25, 0.25])])
            .collect(),
        drills: vec![bench_drill([-10.0, -10.0], 2.0, false)],
        board_outline: None,
        panel_features: None,
    };
    let mounting_hole_keepout_elapsed = time("mounting_hole_keepout_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = mounting_hole_copper_keepout_readiness(
                &mounting_hole_keepout_board,
                &[],
                0.5,
                1.0e-9,
            );
        }
    });
    let mounting_hole_edge_board = BoardModel {
        source: "bench".to_string(),
        copper: Vec::new(),
        drills: (0..1_000)
            .map(|index| {
                bench_drill(
                    [
                        5.0 + (index % 50) as f64 * 0.05,
                        5.0 + (index / 50) as f64 * 0.05,
                    ],
                    2.0,
                    false,
                )
            })
            .chain([bench_drill([1.0, 5.0], 2.0, false)])
            .collect(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
            Some(LayerMetadata {
                name: "bench mounting outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let mounting_hole_edge_elapsed = time("mounting_hole_edge_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = mounting_hole_edge_clearance_readiness(&mounting_hole_edge_board, 0.5, 1.0e-9);
        }
    });
    let mounting_hole_plating_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    "GND",
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    [0.25, 0.25],
                )
            })
            .collect(),
        drills: vec![bench_net_drill("MOUNT", [-10.0, -10.0], 3.2, true)],
        board_outline: None,
        panel_features: None,
    };
    let mounting_hole_plating_elapsed = time("mounting_hole_plating_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = mounting_hole_plating_intent_readiness(&mounting_hole_plating_board, &[], 1.0);
        }
    });
    let gold_finger_spacing_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    &format!("GOLD_FINGER_{index}"),
                    [100.0 + index as f64 * 5.0, 0.0],
                    [0.50, 2.0],
                )
            })
            .chain([
                bench_pad("GOLD_FINGER_A", [0.25, 0.0], [0.50, 2.0]),
                bench_pad("GOLD_FINGER_B", [0.775, 0.0], [0.45, 2.0]),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let gold_finger_spacing_elapsed = time("gold_finger_spacing_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = gold_finger_spacing_readiness(&gold_finger_spacing_board, &[], 0.10, 1.0e-9);
        }
    });
    let gold_finger_intent_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| bench_via("SIG", [100.0 + index as f64 * 2.0, 50.0], 0.20))
            .chain([bench_via("EDGE_CONN_1", [0.0, 0.0], 0.20)])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let gold_finger_intent_elapsed = time("gold_finger_intent_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = gold_finger_readiness(&gold_finger_intent_board, &[]);
        }
    });
    let gold_finger_edge_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                let x = 2.0 + (index % 100) as f64 * 0.10;
                let y = 2.0 + (index / 100) as f64 * 0.10;
                bench_pad("SIG", [x, y], [0.04, 0.04])
            })
            .chain([bench_pad("GOLD_FINGER_CENTER", [9.5, 9.5], [1.0, 1.0])])
            .collect(),
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([10.0, 10.0], [20.0, 20.0], 0.0)],
            Some(LayerMetadata {
                name: "bench gold finger outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let gold_finger_edge_elapsed = time("gold_finger_edge_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = gold_finger_edge_readiness(&gold_finger_edge_board, &[], 1.0);
        }
    });
    let gold_finger_keepout_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    &format!("GOLD_FINGER_{index}"),
                    [100.0 + index as f64 * 5.0, 0.0],
                    [0.50, 2.0],
                )
            })
            .chain([bench_pad("GOLD_FINGER_NEAR", [0.5, 0.0], [1.0, 2.0])])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let gold_finger_keepout_drills = vec![bench_drill([1.25, 0.0], 0.6, false)];
    let gold_finger_keepout_elapsed = time("gold_finger_drill_keepout_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = gold_finger_drill_keepout_readiness(
                &gold_finger_keepout_board,
                &gold_finger_keepout_drills,
                &[],
                0.4,
                1.0e-9,
            );
        }
    });
    let edge_plating_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..1_000)
            .map(|index| {
                bench_pad(
                    &format!("SIG{index}"),
                    [
                        2.0 + (index % 50) as f64 * 0.1,
                        2.0 + (index / 50) as f64 * 0.1,
                    ],
                    [0.02, 0.02],
                )
            })
            .chain([bench_pad("EDGE_PLATING", [0.25, 5.0], [0.2, 0.2])])
            .collect(),
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
            Some(LayerMetadata {
                name: "bench edge plating outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let edge_plating_elapsed = time("edge_plating_rect_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = edge_plating_intent_readiness(&edge_plating_board, &[], 0.5, 1.0e-9);
        }
    });

    let panel_feature_outline_board = BoardModel {
        source: "bench".to_string(),
        copper: Vec::new(),
        drills: Vec::new(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
            Some(LayerMetadata {
                name: "bench panel outline".to_string(),
            }),
        )),
        panel_features: Some(polygons_to_sketch(
            (0..1_000)
                .map(|index| {
                    rect_polygon(
                        [
                            0.10 + (index % 50) as f64 * 0.001,
                            0.10 + (index / 50) as f64 * 0.001,
                        ],
                        [0.05, 0.05],
                        0.0,
                    )
                })
                .chain([rect_polygon([5.0, 5.0], [0.5, 0.5], 0.0)])
                .collect(),
            Some(LayerMetadata {
                name: "bench panel features".to_string(),
            }),
        )),
    };
    let panel_feature_outline_elapsed = time("panel_feature_outline_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = panel_feature_outline_readiness(&panel_feature_outline_board, 0.5, 1.0e-9);
        }
    });

    let castellation_pitch_board = BoardModel {
        source: "bench".to_string(),
        copper: Vec::new(),
        drills: (0..1_000)
            .map(|index| bench_drill([0.0, 20.0 + index as f64 * 4.0], 0.6, true))
            .chain([
                bench_drill([0.0, 3.0], 0.6, true),
                bench_drill([0.0, 3.7], 0.6, true),
            ])
            .collect(),
        board_outline: Some(polygons_to_sketch(
            vec![rect_polygon([5.0, 2_000.0], [10.0, 4_100.0], 0.0)],
            Some(LayerMetadata {
                name: "bench castellation outline".to_string(),
            }),
        )),
        panel_features: None,
    };
    let castellation_pitch_elapsed = time("castellation_pitch_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = castellation_pitch_readiness(&castellation_pitch_board, 0.5);
        }
    });

    let short_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_pad("A", [0.0, 0.0], [0.8, 0.8]),
            bench_pad("B", [0.3, 0.0], [0.8, 0.8]),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let short_elapsed = time("different_net_short_10k", || {
        for _ in 0..10_000 {
            let _ = different_net_short_readiness(&short_board, &[], 1.0e-9);
        }
    });

    let net_usage_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_segment("PCIE_RX0", [0.0, 0.0], [1.0, 0.0], 0.10),
            bench_segment_on_layer("B.Cu", "PCIE_RX0", [2.0, 0.0], [3.0, 0.0], 0.10),
            bench_segment("USB3_DP", [0.0, 1.0], [1.0, 1.0], 0.10),
            bench_segment("VBUS", [0.0, 2.0], [1.0, 2.0], 0.20),
            bench_segment_on_layer("B.Cu", "VBUS", [2.0, 2.0], [3.0, 2.0], 0.20),
            bench_via("VBUS", [1.5, 2.0], 0.25),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let controlled_impedance_elapsed = time("controlled_impedance_10k", || {
        for _ in 0..10_000 {
            let _ = controlled_impedance_readiness(&net_usage_board, &[]);
        }
    });
    let differential_pair_presence_elapsed = time("differential_pair_presence_10k", || {
        for _ in 0..10_000 {
            let _ = differential_pair_readiness(&net_usage_board, &[]);
        }
    });
    let reference_plane_elapsed = time("reference_plane_presence_10k", || {
        for _ in 0..10_000 {
            let _ = reference_plane_readiness(&net_usage_board, &[]);
        }
    });
    let high_current_elapsed = time("high_current_layer_change_10k", || {
        for _ in 0..10_000 {
            let _ = high_current_readiness(&net_usage_board, &[]);
        }
    });
    let intra_pair_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: [
            bench_segment("USB_D+", [0.0, 0.0], [1.0, 0.0], 0.10),
            bench_segment("USB_D-", [0.0, 0.20], [1.0, 0.20], 0.10),
        ]
        .into_iter()
        .chain((0..2_000).flat_map(|index| {
            let x = 100.0 + index as f64 * 2.0;
            [
                bench_segment("USB_D+", [x, 10.0], [x + 0.5, 10.0], 0.10),
                bench_segment("USB_D-", [x, 20.0], [x + 0.5, 20.0], 0.10),
            ]
        }))
        .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let intra_pair_sparse_elapsed = time("differential_pair_spacing_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = differential_pair_spacing_readiness(&intra_pair_sparse_board, &[], 0.30);
        }
    });

    let differential_pair_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_segment("USB1_DP", [0.0, 0.00], [2.0, 0.00], 0.10),
            bench_segment("USB1_DM", [0.0, 0.20], [2.0, 0.20], 0.10),
            bench_segment("USB2_DP", [0.0, 0.50], [2.0, 0.50], 0.10),
            bench_segment("USB2_DM", [0.0, 0.70], [2.0, 0.70], 0.10),
            bench_via("USB1_DP", [1.0, 0.00], 0.20),
            bench_via("USB1_DM", [1.1, 0.00], 0.20),
            bench_via("GND", [1.05, 0.00], 0.20),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let pair_to_pair_elapsed = time("differential_pair_to_pair_spacing_10k", || {
        for _ in 0..10_000 {
            let _ =
                differential_pair_to_pair_spacing_readiness(&differential_pair_board, &[], 0.40);
        }
    });
    let pair_to_pair_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_segment(
                    &format!("PAIR{index}_DP"),
                    [100.0 + index as f64 * 2.0, 20.0],
                    [100.5 + index as f64 * 2.0, 20.0],
                    0.10,
                )
            })
            .chain([
                bench_segment("LANE1_DP", [0.0, 0.0], [2.0, 0.0], 0.10),
                bench_segment("LANE2_DP", [0.0, 0.25], [2.0, 0.25], 0.10),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let pair_to_pair_sparse_elapsed = time("differential_pair_to_pair_spacing_sparse_1k", || {
        for _ in 0..1_000 {
            let _ =
                differential_pair_to_pair_spacing_readiness(&pair_to_pair_sparse_board, &[], 0.30);
        }
    });
    let pair_skew_elapsed = time("differential_pair_skew_10k", || {
        for _ in 0..10_000 {
            let _ = differential_pair_skew_readiness(&differential_pair_board, &[], 0.20);
        }
    });
    let pair_width_elapsed = time("differential_pair_width_10k", || {
        for _ in 0..10_000 {
            let _ = differential_pair_width_readiness(&differential_pair_board, &[], 0.08, 0.04);
        }
    });
    let pair_neckdown_elapsed = time("differential_pair_neckdown_10k", || {
        for _ in 0..10_000 {
            let _ = differential_pair_neckdown_readiness(&differential_pair_board, &[], 0.08, 0.50);
        }
    });
    let pair_via_proximity_elapsed = time("differential_pair_via_proximity_10k", || {
        for _ in 0..10_000 {
            let _ = differential_pair_via_proximity_readiness(&differential_pair_board, &[], 0.20);
        }
    });
    let pair_via_return_elapsed = time("differential_pair_via_return_10k", || {
        for _ in 0..10_000 {
            let _ = differential_pair_via_return_readiness(&differential_pair_board, &[], 0.20);
        }
    });
    let mut dense_pair_vias = Vec::new();
    for index in 0..1_000 {
        let x = index as f64 * 0.25;
        dense_pair_vias.push(bench_via("DDR_DQS_DP", [x, 0.0], 0.20));
        dense_pair_vias.push(bench_via("DDR_DQS_DM", [x + 0.05, 0.0], 0.20));
    }
    let dense_pair_via_board = BoardModel {
        source: "bench".to_string(),
        copper: dense_pair_vias,
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let pair_via_proximity_dense_elapsed = time("differential_pair_via_proximity_dense_1k", || {
        for _ in 0..1_000 {
            let _ = differential_pair_via_proximity_readiness(&dense_pair_via_board, &[], 0.10);
        }
    });
    let mut sparse_ground_vias = vec![
        bench_via("USB_DP", [0.0, 0.0], 0.20),
        bench_via("USB_DM", [0.05, 0.0], 0.20),
    ];
    for index in 0..2_000 {
        sparse_ground_vias.push(bench_via("GND", [100.0 + index as f64 * 0.25, 10.0], 0.20));
    }
    let sparse_ground_board = BoardModel {
        source: "bench".to_string(),
        copper: sparse_ground_vias,
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let pair_via_return_sparse_elapsed = time("differential_pair_via_return_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = differential_pair_via_return_readiness(&sparse_ground_board, &[], 0.20);
        }
    });
    let mut sparse_pair_return_copper = vec![
        bench_segment("USB_D+", [0.0, 0.0], [1.0, 0.0], 0.10),
        bench_segment("USB_D-", [0.0, 0.20], [1.0, 0.20], 0.10),
    ];
    for index in 0..2_000 {
        sparse_pair_return_copper.push(bench_via("GND", [100.0 + index as f64 * 0.50, 10.0], 0.20));
    }
    let sparse_pair_return_board = BoardModel {
        source: "bench".to_string(),
        copper: sparse_pair_return_copper,
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let pair_return_sparse_elapsed = time("differential_pair_return_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = differential_pair_return_readiness(&sparse_pair_return_board, &[], 0.30);
        }
    });

    let split_plane_board = BoardModel {
        source: "bench".to_string(),
        copper: vec![
            bench_zone("GND", [-1.25, 0.0], [1.5, 1.0]),
            bench_zone("GND", [1.25, 0.0], [1.5, 1.0]),
            bench_segment("USB_DP", [-2.0, 0.0], [2.0, 0.0], 0.10),
        ],
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let split_plane_elapsed = time("split_plane_crossing_10k", || {
        for _ in 0..10_000 {
            let _ = split_plane_crossing_readiness(&split_plane_board, &[], 0.05, 1.0e-9);
        }
    });
    let split_plane_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: std::iter::once(bench_segment("USB_DP", [-2.0, 0.0], [2.0, 0.0], 0.10))
            .chain((0..2_000).map(|index| {
                bench_zone(
                    "GND",
                    [
                        100.0 + (index % 100) as f64 * 3.0,
                        100.0 + (index / 100) as f64 * 3.0,
                    ],
                    [1.0, 1.0],
                )
            }))
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let split_plane_sparse_elapsed = time("split_plane_crossing_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = split_plane_crossing_readiness(&split_plane_sparse_board, &[], 0.05, 1.0e-9);
        }
    });
    let return_path_proximity_elapsed = time("return_path_proximity_10k", || {
        for _ in 0..10_000 {
            let _ = return_path_proximity_readiness(&split_plane_board, &[], 0.50);
        }
    });
    let mut sparse_return_path_copper = vec![bench_segment("USB_DP", [0.0, 0.0], [1.0, 0.0], 0.10)];
    for index in 0..2_000 {
        sparse_return_path_copper.push(bench_segment(
            "GND",
            [100.0 + index as f64 * 3.0, 100.0],
            [101.0 + index as f64 * 3.0, 100.0],
            0.10,
        ));
    }
    let sparse_return_path_board = BoardModel {
        source: "bench".to_string(),
        copper: sparse_return_path_copper,
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let return_path_proximity_sparse_elapsed = time("return_path_proximity_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = return_path_proximity_readiness(&sparse_return_path_board, &[], 0.50);
        }
    });
    let mut sparse_reference_plane_copper =
        vec![bench_segment("USB_DP", [0.0, 0.0], [1.0, 0.0], 0.10)];
    for index in 0..2_000 {
        sparse_reference_plane_copper.push(bench_zone(
            "GND",
            [
                100.0 + (index % 100) as f64 * 3.0,
                100.0 + (index / 100) as f64 * 3.0,
            ],
            [0.5, 0.5],
        ));
    }
    sparse_reference_plane_copper.push(bench_zone("GND", [0.5, 0.0], [2.0, 1.0]));
    let sparse_reference_plane_board = BoardModel {
        source: "bench".to_string(),
        copper: sparse_reference_plane_copper,
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let reference_plane_void_sparse_elapsed = time("reference_plane_void_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = reference_plane_void_readiness(&sparse_reference_plane_board, &[], 1.0e-9);
        }
    });
    let orphaned_zone_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| {
                bench_via(
                    "GND",
                    [
                        100.0 + (index % 100) as f64 * 3.0,
                        100.0 + (index / 100) as f64 * 3.0,
                    ],
                    0.10,
                )
            })
            .chain([
                bench_zone("GND", [0.0, 0.0], [2.0, 2.0]),
                bench_via("GND", [0.0, 0.0], 0.12),
            ])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let orphaned_zone_sparse_elapsed = time("orphaned_zone_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = orphaned_zone_readiness(&orphaned_zone_sparse_board, &[], 0.10);
        }
    });
    let return_path_stitching_sparse_board = BoardModel {
        source: "bench".to_string(),
        copper: (0..2_000)
            .map(|index| bench_via("GND", [100.0 + index as f64 * 2.0, 50.0], 0.12))
            .chain([bench_via("USB_D_P", [0.0, 0.0], 0.12)])
            .collect(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let return_path_stitching_sparse_elapsed = time("return_path_stitching_sparse_1k", || {
        for _ in 0..1_000 {
            let _ = return_path_readiness(&return_path_stitching_sparse_board, 0.50, &[]);
        }
    });

    let artifact_bom = TextArtifact {
        path: "bench_bom.csv".to_string(),
        text: "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Polarity,MSL,Height,Side\nD1,1,LED0603,LED,0603 LED,LiteOn,SKU-D,Active,ALT-D,Cathode mark,1,0.8mm,Top\nU1,1,MCU123,MCU,QFN32,Vendor,SKU-U,Active,ALT-U,Pin 1 dot,3,0.9mm,Top\nU2,1,COB-DIE-1,Bare die wire bond sensor,chip-on-board bond pad array,Vendor,SKU-WB,Active,ALT-WB,Die corner 1,3,0.4mm,Top\nU3,1,BUCK-10A,10A buck regulator with exposed pad,QFN32 thermal pad,Vendor,SKU-TH,Active,ALT-TH,Pin 1 dot,3,0.9mm,Top\nJ1,1,USB-C,USB connector,USB-C,Vendor,SKU-J,Active,ALT-J,Pin 1 shell,1,8.0mm,Top\nJ2,1,PF-100,Press-fit compliant pin connector,2x20 press-fit,Vendor,SKU-PF,Active,ALT-PF,Pin 1 mark,1,12.0mm,Top\n".to_string(),
    };
    let artifact_centroid = TextArtifact {
        path: "bench_centroid.csv".to_string(),
        text: "Designator,X,Y,Rotation,Side,Value,Footprint\nD1,1.0,2.0,90,Top,LED,0603 LED\nU1,5.0,6.0,0,Top,MCU,QFN32\nU2,7.0,6.0,0,Top,Bare die wire bond sensor,chip-on-board bond pad array\nU3,8.0,6.0,0,Top,10A buck regulator with exposed pad,QFN32 thermal pad\nJ1,9.0,2.0,180,Top,USB connector,USB-C\nJ2,12.0,2.0,0,Top,Press-fit compliant pin connector,2x20 press-fit\n".to_string(),
    };
    let artifact_readme = TextArtifact {
        path: "README.md".to_string(),
        text: "Revision bench package. Stackup: 4 layer, 1.6mm board thickness, 1 oz copper weight. Finish: ENEPIG soft gold for wire bond pads. Soldermask: green. Controlled impedance: no impedance. Panelization: no panel. Via treatment: tented vias. Edge plating: no edge plating. Castellations: no castellation. Date code and revision text use label location in fab drawing. Preflight: DRC/ERC passed, zones refilled, outputs generated and reloaded in Gerber viewer. HyperDRC reviewed with no waivers. Submitted package archived. Assembly: pin-1 and polarity reviewed against assembly drawing. X-ray inspection for QFN. Selective solder and hand solder process notes cover the USB connector. Component height and enclosure clearance reviewed against mechanical keepout. Thermal validation: temperature rise, derating, airflow, thermal vias, and heat spreader reviewed. Press-fit process: compliant-pin insertion force, finished-hole tolerance, press tooling, and support fixture reviewed. Wire bonding: die attach, bond map, loop height, and bond pull test handoff reviewed. Packaging: MSL dry pack with desiccant, humidity card, moisture barrier bag, ESD bag, and lot label. Reflow profile: validated oven recipe with soak, peak temperature, ramp rate, and time above liquidus. Cleanliness: no-clean flux residue and ionic contamination controls reviewed for low-standoff packages.".to_string(),
    };
    let artifact_fab = FileArtifact {
        path: "bench_fab.pdf".to_string(),
        byte_len: 256,
    };
    let artifact_assembly = FileArtifact {
        path: "bench_assembly.pdf".to_string(),
        byte_len: 256,
    };
    let artifact_elapsed = time("production_artifact_readiness_5k", || {
        for _ in 0..5_000 {
            let _ = production_artifact_readiness(
                std::slice::from_ref(&artifact_bom),
                std::slice::from_ref(&artifact_centroid),
                &[],
                std::slice::from_ref(&artifact_readme),
                std::slice::from_ref(&artifact_fab),
                std::slice::from_ref(&artifact_assembly),
                &[],
            );
        }
    });
    let waiver_inputs = (0..100)
        .map(|index| Waiver {
            id: None,
            check: Some(format!("check-{index}")),
            layers: Vec::new(),
            message_contains: None,
            reason: Some("accepted for release review".to_string()),
            owner: Some("manufacturing".to_string()),
            review_date: Some("2099-12-31".to_string()),
            source: Some(format!("ECO-{index:04}")),
            geometry_hash: Some(format!("hash-{index:04}")),
        })
        .collect::<Vec<_>>();
    let waiver_governance_elapsed = time("waiver_governance_10k", || {
        for _ in 0..10_000 {
            let _ = governance_violations(&waiver_inputs);
        }
    });
    let stub_violations = (0..100)
        .map(|index| {
            Violation::new(
                "waiver-bench",
                Severity::Warning,
                vec!["F.Cu".to_string()],
                Some(index),
                Vec::new(),
                vec![[index as f64 * 0.01, index as f64 * 0.02]],
                Some("waiver benchmark finding".to_string()),
            )
        })
        .collect::<Vec<_>>();
    let stub_report = Report {
        files: Vec::new(),
        inputs: Vec::new(),
        diagnostics: Vec::new(),
        violation_count: stub_violations.len(),
        waived_count: 0,
        summary: report_summary(&stub_violations, 0),
        violations: stub_violations,
    };
    let waiver_stub_elapsed = time("waiver_stub_fingerprint_10k", || {
        for _ in 0..10_000 {
            let _ = report_to_waiver_stubs(&stub_report);
        }
    });

    println!(
        "parser_geometry_smoke total_ms={:.3}",
        (parse_elapsed
            + geometry_elapsed
            + duplicate_layer_elapsed
            + duplicate_island_elapsed
            + tiny_feature_elapsed
            + skinny_feature_elapsed
            + density_elapsed
            + copper_width_elapsed
            + copper_net_intent_elapsed
            + paste_ratio_sparse_elapsed
            + paste_coverage_sparse_elapsed
            + paste_overhang_sparse_elapsed
            + mask_coverage_sparse_elapsed
            + exposed_copper_sparse_elapsed
            + mask_expansion_sparse_elapsed
            + paste_mask_alignment_sparse_elapsed
            + mask_overlap_clearance_sparse_elapsed
            + paste_spacing_sparse_elapsed
            + mask_spacing_sparse_elapsed
            + mask_island_sparse_elapsed
            + silkscreen_overlap_sparse_elapsed
            + silkscreen_clearance_sparse_elapsed
            + tombstone_elapsed
            + paste_via_elapsed
            + thermal_pad_windowpane_sparse_elapsed
            + net_constraint_elapsed
            + net_constraint_pair_elapsed
            + different_net_spacing_elapsed
            + registration_elapsed
            + acid_trap_elapsed
            + acid_trap_sparse_elapsed
            + via_in_pad_elapsed
            + teardrop_elapsed
            + local_fiducial_elapsed
            + dense_pad_escape_elapsed
            + dense_pad_via_elapsed
            + dense_pad_via_sparse_pads_elapsed
            + dense_pad_mask_elapsed
            + dense_pad_mask_sparse_elapsed
            + component_spacing_elapsed
            + component_edge_elapsed
            + component_hole_elapsed
            + connector_rework_elapsed
            + connector_return_sparse_elapsed
            + edge_stitching_sparse_elapsed
            + board_edge_exposure_elapsed
            + high_speed_edge_elapsed
            + high_voltage_edge_elapsed
            + chassis_stitching_sparse_elapsed
            + pad_pair_asymmetry_elapsed
            + fiducial_keepout_elapsed
            + fiducial_edge_elapsed
            + selective_wave_elapsed
            + press_fit_elapsed
            + mouse_bite_elapsed
            + tooling_hole_elapsed
            + conformal_coating_elapsed
            + testpoint_access_elapsed
            + testpoint_coverage_elapsed
            + testpoint_side_elapsed
            + testpoint_copper_elapsed
            + antenna_keepout_elapsed
            + rf_keepout_elapsed
            + rf_via_fence_elapsed
            + inductor_keepout_elapsed
            + switch_node_elapsed
            + power_plane_elapsed
            + high_current_neck_elapsed
            + power_pad_entry_elapsed
            + power_pad_entry_sparse_elapsed
            + power_via_return_elapsed
            + power_via_return_sparse_elapsed
            + power_via_array_sparse_elapsed
            + decoupling_sparse_elapsed
            + thermal_via_distribution_elapsed
            + thermal_via_cluster_elapsed
            + thermal_via_elapsed
            + thermal_via_distribution_sparse_elapsed
            + thermal_copper_area_elapsed
            + thermal_relief_elapsed
            + thermal_pad_via_elapsed
            + hot_component_spacing_elapsed
            + thermal_mechanical_elapsed
            + esd_return_path_elapsed
            + esd_protection_elapsed
            + protective_earth_spacing_elapsed
            + voltage_clearance_elapsed
            + surge_keepout_elapsed
            + mixed_signal_partition_elapsed
            + sensitive_spacing_elapsed
            + sensitive_return_elapsed
            + min_copper_neck_elapsed
            + continuity_elapsed
            + continuity_sparse_drills_elapsed
            + same_net_island_sparse_elapsed
            + plane_clearance_sparse_elapsed
            + panelization_clearance_elapsed
            + drill_clearance_elapsed
            + plating_intent_elapsed
            + board_outline_drill_elapsed
            + drill_spacing_elapsed
            + drill_table_elapsed
            + ipc356_apply_elapsed
            + ipc356_coverage_elapsed
            + ipc356_drill_elapsed
            + mounting_hole_spacing_elapsed
            + mounting_hole_distribution_elapsed
            + mounting_hole_grounding_elapsed
            + mounting_hole_keepout_elapsed
            + mounting_hole_edge_elapsed
            + mounting_hole_plating_elapsed
            + gold_finger_spacing_elapsed
            + gold_finger_intent_elapsed
            + gold_finger_edge_elapsed
            + gold_finger_keepout_elapsed
            + edge_plating_elapsed
            + panel_feature_outline_elapsed
            + castellation_pitch_elapsed
            + short_elapsed
            + controlled_impedance_elapsed
            + differential_pair_presence_elapsed
            + reference_plane_elapsed
            + high_current_elapsed
            + intra_pair_sparse_elapsed
            + pair_to_pair_elapsed
            + pair_to_pair_sparse_elapsed
            + pair_skew_elapsed
            + pair_width_elapsed
            + pair_neckdown_elapsed
            + pair_via_proximity_elapsed
            + pair_via_return_elapsed
            + pair_via_proximity_dense_elapsed
            + pair_via_return_sparse_elapsed
            + pair_return_sparse_elapsed
            + split_plane_elapsed
            + split_plane_sparse_elapsed
            + return_path_proximity_elapsed
            + return_path_proximity_sparse_elapsed
            + reference_plane_void_sparse_elapsed
            + orphaned_zone_sparse_elapsed
            + return_path_stitching_sparse_elapsed
            + artifact_elapsed
            + waiver_governance_elapsed
            + waiver_stub_elapsed)
            .as_secs_f64()
            * 1000.0
    );
}

fn bench_testpoint(net: &str, location: [f64; 2], diameter: f64) -> Ipc356Point {
    Ipc356Point {
        net: net.to_string(),
        reference: Some(net.to_string()),
        pin: Some("1".to_string()),
        location,
        diameter: Some(diameter),
        access_side: Some(Ipc356AccessSide::Top),
        feature_type: Some(Ipc356FeatureType::Smd),
        soldermask: Some(Ipc356Soldermask::Open),
    }
}

fn bench_segment(net: &str, start: [f64; 2], end: [f64; 2], width: f64) -> CopperFeature {
    bench_segment_on_layer("F.Cu", net, start, end, width)
}

fn bench_segment_on_layer(
    layer: &str,
    net: &str,
    start: [f64; 2],
    end: [f64; 2],
    width: f64,
) -> CopperFeature {
    CopperFeature {
        layer: layer.to_string(),
        net: Some(net.to_string()),
        kind: CopperKind::Segment,
        location: [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0],
        sketch: polygons_to_sketch(
            vec![line_polygon(start, end, width).expect("benchmark segment should be valid")],
            Some(LayerMetadata {
                name: "bench segment".to_string(),
            }),
        ),
    }
}

fn bench_dense_pad_cluster() -> Vec<CopperFeature> {
    let mut copper = bench_dense_pad_cluster_with_size(0.25);
    copper.push(bench_via("ESC", [0.32, 0.0], 0.20));
    copper
}

fn bench_dense_pad_cluster_with_size(size: f64) -> Vec<CopperFeature> {
    let mut copper = Vec::new();
    for x in 0..4 {
        for y in 0..4 {
            copper.push(bench_pad(
                &format!("BGA_{x}_{y}"),
                [x as f64 * 0.5, y as f64 * 0.5],
                [size, size],
            ));
        }
    }
    copper
}

fn bench_pad(net: &str, location: [f64; 2], size: [f64; 2]) -> CopperFeature {
    bench_pad_on_layer("F.Cu", net, location, size)
}

fn bench_unnetted_pad(location: [f64; 2], size: [f64; 2]) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".to_string(),
        net: None,
        kind: CopperKind::Pad,
        location,
        sketch: polygons_to_sketch(
            vec![rect_polygon(location, size, 0.0)],
            Some(LayerMetadata {
                name: "bench unnetted pad".to_string(),
            }),
        ),
    }
}

fn bench_pad_on_layer(layer: &str, net: &str, location: [f64; 2], size: [f64; 2]) -> CopperFeature {
    CopperFeature {
        layer: layer.to_string(),
        net: Some(net.to_string()),
        kind: CopperKind::Pad,
        location,
        sketch: polygons_to_sketch(
            vec![rect_polygon(location, size, 0.0)],
            Some(LayerMetadata {
                name: "bench pad".to_string(),
            }),
        ),
    }
}

fn bench_fiducial(location: [f64; 2], diameter: f64) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".to_string(),
        net: None,
        kind: CopperKind::Pad,
        location,
        sketch: polygons_to_sketch(
            vec![rect_polygon(location, [diameter, diameter], 0.0)],
            Some(LayerMetadata {
                name: "bench fiducial".to_string(),
            }),
        ),
    }
}

fn bench_via(net: &str, location: [f64; 2], diameter: f64) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".to_string(),
        net: Some(net.to_string()),
        kind: CopperKind::Via,
        location,
        sketch: polygons_to_sketch(
            vec![circle_polygon(location, diameter / 2.0, 32)],
            Some(LayerMetadata {
                name: "bench via".to_string(),
            }),
        ),
    }
}

fn bench_drill(location: [f64; 2], diameter: f64, plated: bool) -> DrillFeature {
    DrillFeature {
        location,
        diameter,
        net: None,
        plated,
    }
}

fn bench_net_drill(net: &str, location: [f64; 2], diameter: f64, plated: bool) -> DrillFeature {
    DrillFeature {
        location,
        diameter,
        net: Some(net.to_string()),
        plated,
    }
}

fn bench_zone(net: &str, location: [f64; 2], size: [f64; 2]) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".to_string(),
        net: Some(net.to_string()),
        kind: CopperKind::Zone,
        location,
        sketch: polygons_to_sketch(
            vec![rect_polygon(location, size, 0.0)],
            Some(LayerMetadata {
                name: "bench zone".to_string(),
            }),
        ),
    }
}

fn time(name: &str, run: impl FnOnce()) -> std::time::Duration {
    let start = Instant::now();
    run();
    let elapsed = start.elapsed();
    println!("{name} ms={:.3}", elapsed.as_secs_f64() * 1000.0);
    elapsed
}
