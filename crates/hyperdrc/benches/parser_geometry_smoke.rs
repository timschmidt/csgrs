use std::time::Instant;

use hyperdrc::LayerMetadata;
use hyperdrc::checks::{
    FileArtifact, TextArtifact, antenna_copper_keepout_readiness, dense_pad_mask_bridge_readiness,
    dense_pad_via_spacing_readiness, different_net_short_readiness,
    differential_pair_neckdown_readiness, differential_pair_skew_readiness,
    differential_pair_to_pair_spacing_readiness, differential_pair_via_proximity_readiness,
    differential_pair_via_return_readiness, differential_pair_width_readiness,
    esd_return_path_readiness, inductor_copper_keepout_readiness, local_copper_density_readiness,
    min_copper_neck_width, mixed_signal_partition_readiness, power_pad_entry_readiness,
    power_via_return_readiness, production_artifact_readiness, protective_earth_spacing_readiness,
    return_path_proximity_readiness, same_net_drill_break_readiness,
    split_plane_crossing_readiness, surge_protection_keepout_readiness,
    thermal_via_distribution_readiness, trace_junction_acid_trap_readiness,
};
use hyperdrc::geometry::{circle_polygon, line_polygon, polygons_to_sketch, rect_polygon};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use hyperdrc::sexp;

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

    println!(
        "parser_geometry_smoke total_ms={:.3}",
        (parse_elapsed
            + geometry_elapsed
            + density_elapsed
            + acid_trap_elapsed
            + dense_pad_via_elapsed
            + dense_pad_mask_elapsed
            + antenna_keepout_elapsed
            + inductor_keepout_elapsed
            + power_pad_entry_elapsed
            + power_via_return_elapsed
            + thermal_via_distribution_elapsed
            + esd_return_path_elapsed
            + protective_earth_spacing_elapsed
            + surge_keepout_elapsed
            + mixed_signal_partition_elapsed
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
            + artifact_elapsed)
            .as_secs_f64()
            * 1000.0
    );
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
