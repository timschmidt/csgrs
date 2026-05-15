use std::time::Instant;

use hyperdrc::LayerMetadata;
use hyperdrc::baseline::report_to_waiver_stubs;
use hyperdrc::checks::{
    FileArtifact, TextArtifact, antenna_copper_keepout_readiness,
    component_hole_clearance_readiness, component_spacing_readiness,
    connector_rework_clearance_readiness, dense_pad_mask_bridge_readiness,
    dense_pad_via_spacing_readiness, different_net_short_readiness,
    differential_pair_neckdown_readiness, differential_pair_skew_readiness,
    differential_pair_to_pair_spacing_readiness, differential_pair_via_proximity_readiness,
    differential_pair_via_return_readiness, differential_pair_width_readiness,
    duplicate_layer_geometry_readiness, duplicate_layer_island_readiness, esd_protection_readiness,
    esd_return_path_readiness, fiducial_keepout_readiness, hot_component_spacing_readiness,
    inductor_copper_keepout_readiness, local_copper_density_readiness, min_copper_neck_width,
    mixed_signal_partition_readiness, pad_pair_asymmetry_readiness, power_pad_entry_readiness,
    power_via_return_readiness, press_fit_keepout_readiness, production_artifact_readiness,
    protective_earth_spacing_readiness, return_path_proximity_readiness, rf_keepout_readiness,
    rf_via_fence_readiness, same_net_drill_break_readiness,
    selective_wave_solder_keepout_readiness, sensitive_net_spacing_readiness,
    sensitive_return_readiness, skinny_layer_feature_readiness, split_plane_crossing_readiness,
    surge_protection_keepout_readiness, switch_node_keepout_readiness,
    testpoint_accessibility_readiness, thermal_copper_area_readiness,
    thermal_mechanical_keepout_readiness, thermal_via_distribution_readiness,
    tiny_layer_feature_readiness, trace_junction_acid_trap_readiness, voltage_clearance_readiness,
};
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

    let dense_pad_board = BoardModel {
        source: "bench".to_string(),
        copper: bench_dense_pad_cluster(),
        drills: Vec::new(),
        board_outline: None,
        panel_features: None,
    };
    let dense_pad_via_elapsed = time("dense_pad_via_spacing_5k", || {
        for _ in 0..5_000 {
            let _ = dense_pad_via_spacing_readiness(&dense_pad_board, &[], 0.8, 2.0, 0.15, 1.0e-9);
        }
    });

    let dense_pad_mask_elapsed = time("dense_pad_mask_bridge_10k", || {
        for _ in 0..10_000 {
            let _ = dense_pad_mask_bridge_readiness(&dense_pad_board, &[], 0.8, 0.10);
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
    let return_path_proximity_elapsed = time("return_path_proximity_10k", || {
        for _ in 0..10_000 {
            let _ = return_path_proximity_readiness(&split_plane_board, &[], 0.50);
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
            + acid_trap_elapsed
            + dense_pad_via_elapsed
            + dense_pad_mask_elapsed
            + component_spacing_elapsed
            + component_hole_elapsed
            + connector_rework_elapsed
            + pad_pair_asymmetry_elapsed
            + fiducial_keepout_elapsed
            + selective_wave_elapsed
            + press_fit_elapsed
            + testpoint_access_elapsed
            + antenna_keepout_elapsed
            + rf_keepout_elapsed
            + rf_via_fence_elapsed
            + inductor_keepout_elapsed
            + switch_node_elapsed
            + power_pad_entry_elapsed
            + power_via_return_elapsed
            + thermal_via_distribution_elapsed
            + thermal_copper_area_elapsed
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
            + short_elapsed
            + pair_to_pair_elapsed
            + pair_skew_elapsed
            + pair_width_elapsed
            + pair_neckdown_elapsed
            + pair_via_proximity_elapsed
            + pair_via_return_elapsed
            + split_plane_elapsed
            + return_path_proximity_elapsed
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
    CopperFeature {
        layer: "F.Cu".to_string(),
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
    let mut copper = Vec::new();
    for x in 0..4 {
        for y in 0..4 {
            copper.push(bench_pad(
                &format!("BGA_{x}_{y}"),
                [x as f64 * 0.5, y as f64 * 0.5],
                [0.25, 0.25],
            ));
        }
    }
    copper.push(bench_via("ESC", [0.32, 0.0], 0.20));
    copper
}

fn bench_pad(net: &str, location: [f64; 2], size: [f64; 2]) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".to_string(),
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
