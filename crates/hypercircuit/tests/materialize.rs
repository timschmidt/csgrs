#![cfg(feature = "geometry")]

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, Circuit, CircuitId, CopperZone, DrillHit, DrillShape,
    KeepoutId, KeepoutScope, MaterializationOptions, Net, NetId, PcbDesignRules, PcbKeepout,
    PcbLayout, PcbRoute, PcbStackup, PcbVia, Plating, Real, RouteId, StackupLayer,
    StackupLayerKind, TransientPolicy, ViaId, ZoneId,
};
use hyperlattice::Point2;
use hyperpath::{LinePathSegment, TraceLayer};

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

#[test]
fn multi_segment_orthogonal_route_materializes_as_one_source_feature() {
    let signal = NetId::new("SIGNAL").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("orthogonal-route").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    });
    let layer = TraceLayer(0);
    let route_id = RouteId::new("serpentine").unwrap();
    let layout = PcbLayout {
        id: BoardId::new("orthogonal-route").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(0, 0), p(20, 0), p(20, 10), p(0, 10)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(layer),
                thickness: Real::one(),
                material: None,
            }],
        },
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: vec![PcbRoute {
            id: route_id.clone(),
            net: signal,
            layer,
            width: Real::one(),
            segments: vec![
                LinePathSegment::new(p(2, 2), p(8, 2)).into(),
                LinePathSegment::new(p(8, 2), p(8, 6)).into(),
                LinePathSegment::new(p(8, 6), p(12, 6)).into(),
                LinePathSegment::new(p(12, 6), p(12, 2)).into(),
                LinePathSegment::new(p(12, 2), p(18, 2)).into(),
            ],
        }],
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };

    let report = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    let route_features = report
        .copper_features
        .iter()
        .filter(|feature| {
            feature.identity == hypercircuit::MaterializedCopperIdentity::Route(route_id.clone())
        })
        .count();
    assert_eq!(route_features, 1);
}

#[test]
fn declarative_board_materializes_source_addressable_copper_and_drills() {
    let signal = NetId::new("SIGNAL").unwrap();
    let foreign = NetId::new("FOREIGN").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("materialized-board").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: foreign.clone(),
        is_ground: false,
    });
    let front = TraceLayer(0);
    let back = TraceLayer(1);
    let layout = PcbLayout {
        id: BoardId::new("main").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(0, 0), p(30, 0), p(30, 20), p(0, 20)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![
                StackupLayer {
                    name: "F.Cu".into(),
                    kind: StackupLayerKind::Conductor(front),
                    thickness: Real::from(1),
                    material: None,
                },
                StackupLayer {
                    name: "Core".into(),
                    kind: StackupLayerKind::Dielectric,
                    thickness: Real::from(1),
                    material: Some("hyperphysics:FR4".into()),
                },
                StackupLayer {
                    name: "B.Cu".into(),
                    kind: StackupLayerKind::Conductor(back),
                    thickness: Real::from(1),
                    material: None,
                },
            ],
        },
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: vec![
            PcbRoute {
                id: RouteId::new("signal-route").unwrap(),
                net: signal.clone(),
                layer: front,
                width: Real::from(2),
                segments: vec![LinePathSegment::new(p(4, 4), p(15, 4)).into()],
            },
            PcbRoute {
                id: RouteId::new("foreign-route").unwrap(),
                net: foreign,
                layer: back,
                width: Real::one(),
                segments: vec![LinePathSegment::new(p(12, 10), p(20, 10)).into()],
            },
        ],
        vias: vec![PcbVia {
            id: ViaId::new("signal-via").unwrap(),
            net: signal.clone(),
            start_layer: front,
            end_layer: back,
            center: p(15, 4),
            land_diameter: Real::from(4),
            drill_diameter: Real::from(2),
            plating: Plating::Plated,
            mask: hypercircuit::ViaMaskIntent::default(),
        }],
        zones: vec![CopperZone {
            id: ZoneId::new("signal-zone").unwrap(),
            net: signal,
            layer: back,
            boundary: vec![p(10, 1), p(25, 1), p(25, 16), p(10, 16)],
            clearance: Real::one(),
            fill: hypercircuit::CopperZoneFill::Hatched {
                line_width: Real::one(),
                gap: Real::one(),
                angle_degrees: Real::zero(),
            },
            connection: hypercircuit::CopperZoneConnection::ThermalRelief {
                air_gap: Real::one(),
                spoke_width: Real::one(),
                spoke_count: 4,
            },
            islands: hypercircuit::CopperZoneIslandPolicy::retain_all(),
            stitching: None,
            priority: 2,
        }],
        keepouts: vec![PcbKeepout {
            id: KeepoutId::new("route-exclusion").unwrap(),
            boundary: vec![p(5, 3), p(10, 3), p(10, 5), p(5, 5)],
            scope: KeepoutScope::Copper(vec![front, back]),
        }],
        rules: PcbDesignRules::default(),
    };

    let report = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    assert_eq!(report.copper_features.len(), 5);
    assert!(report.copper_features.iter().any(|feature| matches!(
        &feature.identity,
        hypercircuit::MaterializedCopperIdentity::Route(route)
            if route.as_str() == "signal-route"
    )));
    assert!(report.copper_features.iter().any(|feature| matches!(
        &feature.identity,
        hypercircuit::MaterializedCopperIdentity::Via(via)
            if via.as_str() == "signal-via"
    )));
    assert!(report.copper_features.iter().any(|feature| matches!(
        &feature.identity,
        hypercircuit::MaterializedCopperIdentity::Zone(zone)
            if zone.as_str() == "signal-zone"
    )));
    assert_eq!(report.copper_layers.len(), 2);
    assert_eq!(report.drills.len(), 1);
    assert_eq!(report.zone_realizations.len(), 1);
    assert_eq!(report.zone_realizations[0].fill, "hatched");
    assert_eq!(report.zone_realizations[0].connection, "thermal-relief");
    assert_eq!(report.zone_realizations[0].cleared_foreign_features, 1);
    assert_eq!(report.zone_realizations[0].treated_same_net_lands, 1);
    assert_eq!(
        report.zone_realizations[0].thermal_bounding_box_projections,
        1
    );
    assert_eq!(report.zone_realizations[0].applied_keepouts, 1);
    let preview = layout
        .to_svg(&circuit, hypercircuit::PcbSvgOptions::default())
        .unwrap();
    assert!(preview.svg.contains("id=\"zone-hatch-0\""));
    assert!(preview.svg.contains("data-connection=\"thermal-relief\""));
    let kicad = layout
        .export_kicad(&circuit, hypercircuit::KiCadExportOptions::default())
        .unwrap();
    assert!(kicad.board.contains("(priority 2)"));
    assert!(kicad.board.contains("(mode hatch)"));
    assert!(kicad.board.contains("(thermal_gap 1)"));
    assert!(!kicad.omissions.iter().any(|omission| matches!(
        omission,
        hypercircuit::KiCadExportOmission::ZoneThermalSpokeCount { .. }
    )));
    #[cfg(feature = "interchange")]
    {
        let document = hypercircuit::SemanticDocument::new(circuit.clone(), None)
            .unwrap()
            .with_pcb(layout.clone())
            .unwrap();
        let json = document.to_json_pretty().unwrap();
        assert!(json.contains("\"Hatched\""));
        assert!(json.contains("\"ThermalRelief\""));
        assert_eq!(
            hypercircuit::SemanticDocument::from_json(&json).unwrap(),
            document
        );
    }
    assert!(
        report
            .copper_features
            .iter()
            .all(|feature| feature.net.is_some())
    );
    assert!(
        report
            .copper_layers
            .iter()
            .all(|image| image.copper.is_some() || image.blocker.is_some())
    );
    let assembly = report.stackup_3d(&layout);
    assert_eq!(assembly.total_thickness, Real::from(3));
    let core = assembly
        .layers
        .iter()
        .find(|layer| layer.metadata.name == "Core")
        .unwrap();
    assert_eq!(core.metadata.z_start, Real::one());
    assert_eq!(core.metadata.thickness, Real::one());
    assert_eq!(
        assembly
            .subtractions
            .iter()
            .filter(|evidence| matches!(
                evidence.kind,
                hypercircuit::Pcb3dSubtractionKind::Drill { .. }
            ))
            .count(),
        1,
        "3D omissions: {:?}",
        assembly.omissions
    );
    assert!(!assembly.omissions.iter().any(|omission| matches!(
        omission,
        hypercircuit::Pcb3dAssemblyOmission::InvalidDrillGeometry { .. }
            | hypercircuit::Pcb3dAssemblyOmission::DrillSubtractionFailed { .. }
    )));
    let gltf = assembly.to_gltf("materialized-board").unwrap();
    let scene = serde_json::from_str::<serde_json::Value>(&gltf.gltf).unwrap();
    assert_eq!(scene["meshes"].as_array().unwrap().len(), 1);
    assert_eq!(gltf.objects.len(), 1);
    assert_eq!(
        gltf.coordinate_encoding,
        hypercircuit::Pcb3dCoordinateEncoding::Ieee754Binary32
    );
    let mut slotted_report = report.clone();
    slotted_report.drills.push(DrillHit {
        source: "connector-slot".into(),
        center: p(2, 2),
        shape: DrillShape::Slot {
            start: p(1, 2),
            end: p(3, 2),
            width: Real::from(1),
        },
        plating: Plating::NonPlated,
    });
    let slotted_assembly = slotted_report.stackup_3d(&layout);
    assert!(
        slotted_assembly
            .subtractions
            .iter()
            .any(|evidence| matches!(
                &evidence.kind,
                hypercircuit::Pcb3dSubtractionKind::Drill {
                    source,
                    plating: Plating::NonPlated,
                } if source == "connector-slot"
            ))
    );
    assert!(!slotted_assembly.omissions.iter().any(|omission| matches!(
        omission,
        hypercircuit::Pcb3dAssemblyOmission::InvalidDrillGeometry { source, .. }
            | hypercircuit::Pcb3dAssemblyOmission::DrillSubtractionFailed { source, .. }
            if source == "connector-slot"
    )));
    #[cfg(feature = "drc")]
    {
        use std::fs;

        let handoff = hypercircuit::HyperDrcHandoff::from_materialization(&layout, &slotted_report);
        assert_eq!(handoff.board.source, "main");
        assert_eq!(handoff.board.copper.len(), 5);
        assert_eq!(handoff.board.drills.len(), 1);
        assert_eq!(handoff.stackup.copper_layer_count, Some(2));
        assert_eq!(handoff.authored_keepouts.len(), 1);
        assert_eq!(handoff.authored_slots.len(), 1);
        let readiness = handoff.run_readiness(&hypercircuit::DrcReadinessPolicy {
            minimum_route_width: Real::from(2),
            ..hypercircuit::DrcReadinessPolicy::default()
        });
        assert!(!readiness.is_release_clean());
        assert!(readiness.violations.iter().any(|violation| {
            violation.check == "authored-keepout-readiness"
                && violation.severity == hyperdrc::Severity::Error
        }));
        assert!(readiness.violations.iter().any(|violation| {
            violation.check == "authored-routed-slot-readiness"
                && violation.severity == hyperdrc::Severity::Warning
        }));

        let export = layout
            .export_kicad(&circuit, hypercircuit::KiCadExportOptions::default())
            .unwrap();
        let path = std::env::temp_dir().join(format!(
            "hypercircuit-roundtrip-{}.kicad_pcb",
            std::process::id()
        ));
        fs::write(&path, export.board).unwrap();
        let reparsed = hyperdrc::kicad::load_kicad_pcb(&path).unwrap();
        fs::remove_file(path).unwrap();
        assert!(!reparsed.copper.is_empty());
        assert_eq!(reparsed.drills.len(), 1);
        assert!(reparsed.board_outline.is_some());
    }
}

#[test]
fn zone_island_policy_prunes_unconnected_and_exact_undersized_components() {
    let ground = NetId::new("GND").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("island-policy").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    });
    let layer = TraceLayer(0);
    let layout = PcbLayout {
        id: BoardId::new("island-policy").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(-1, -1), p(21, -1), p(21, 11), p(-1, 11)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(layer),
                thickness: Real::one(),
                material: None,
            }],
        },
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: vec![PcbRoute {
            id: RouteId::new("ground-anchor").unwrap(),
            net: ground.clone(),
            layer,
            width: Real::one(),
            segments: vec![LinePathSegment::new(p(5, 5), p(7, 5)).into()],
        }],
        vias: Vec::new(),
        zones: vec![CopperZone {
            id: ZoneId::new("ground-pour").unwrap(),
            net: ground,
            layer,
            boundary: vec![p(0, 0), p(20, 0), p(20, 10), p(0, 10)],
            clearance: Real::zero(),
            fill: hypercircuit::CopperZoneFill::Solid,
            connection: hypercircuit::CopperZoneConnection::Solid,
            islands: hypercircuit::CopperZoneIslandPolicy::remove_unconnected()
                .with_minimum_area(Real::from(40)),
            stitching: None,
            priority: 0,
        }],
        keepouts: vec![
            PcbKeepout {
                id: KeepoutId::new("left-split").unwrap(),
                boundary: vec![p(3, -1), p(4, -1), p(4, 11), p(3, 11)],
                scope: KeepoutScope::Copper(vec![layer]),
            },
            PcbKeepout {
                id: KeepoutId::new("right-split").unwrap(),
                boundary: vec![p(9, -1), p(11, -1), p(11, 11), p(9, 11)],
                scope: KeepoutScope::Copper(vec![layer]),
            },
        ],
        rules: PcbDesignRules::default(),
    };

    let report = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    let evidence = &report.zone_realizations[0];
    assert_eq!(evidence.initial_islands, 3);
    assert_eq!(evidence.retained_islands, 2);
    assert_eq!(evidence.pruned_unconnected_islands, 1);
    assert_eq!(evidence.pruned_below_area_islands, 1);
    let zone = report
        .copper_features
        .iter()
        .find(|feature| feature.source == "zone:ground-pour")
        .unwrap();
    assert_eq!(
        zone.profile.contains_xy(Real::from(6), Real::from(5)),
        Some(true)
    );
    assert_eq!(
        zone.profile.contains_xy(Real::from(1), Real::from(5)),
        Some(false)
    );
    assert_eq!(
        zone.profile.contains_xy(Real::from(15), Real::from(5)),
        Some(true)
    );
    let preview = layout
        .to_svg(&circuit, hypercircuit::PcbSvgOptions::default())
        .unwrap();
    assert!(
        preview
            .svg
            .contains("data-island-mode=\"remove-unconnected-below-area\"")
    );
    assert!(preview.svg.contains("data-island-area-min=\"40\""));
    let kicad = layout
        .export_kicad(&circuit, hypercircuit::KiCadExportOptions::default())
        .unwrap();
    assert!(kicad.board.contains("(island_removal_mode 2)"));
    assert!(kicad.board.contains("(island_area_min 40)"));
    #[cfg(feature = "interchange")]
    {
        let document = hypercircuit::SemanticDocument::new(circuit.clone(), None)
            .unwrap()
            .with_pcb(layout.clone())
            .unwrap();
        let json = document.to_json_pretty().unwrap();
        assert!(json.contains("\"minimum_area\""));
        assert_eq!(
            hypercircuit::SemanticDocument::from_json(&json).unwrap(),
            document
        );
    }
}
