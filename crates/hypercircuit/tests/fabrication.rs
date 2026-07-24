#![cfg(feature = "geometry")]

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, BoardSide, Circuit, CircuitId, CircuitInstance,
    CircuitInstanceId, ComponentId, CopperFeatureKind, DeviceModel, DeviceModelId, DeviceModelKind,
    DevicePin, DrillHit, DrillShape, FabricationExportOptions, FabricationFileKind,
    FabricationIntegrityIssue, FabricationLengthUnit, FabricationManifest, FabricationPackage,
    LandPattern, LandPatternGraphic, LandPatternGraphicId, LandPatternGraphicPrimitive,
    LandPatternId, LandPatternPad, LayerRole, MaterializationOptions, Net, NetId, PadId, PadPinMap,
    PadShape, PcbDesignRules, PcbLayout, PcbPlacement, PcbStackup, PcbVia, PinBinding,
    PinElectricalKind, PinRef, Plating, ProcessLayerRole, ProcessMaterializationOmission,
    ProductionTextPolicy, Real, StackupLayer, StackupLayerKind, TransientPolicy, ViaId,
    ZoneMaterializationEvidence,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn demo_font() -> Vec<u8> {
    // Compact ttf-parser demo fixture containing one outlined `A` glyph.
    const HEX: &str = "000100000007004000020030636d617000090076000001000000002c676c7966f1cb6698000001340000005c68656164f235ddf80000007c0000003668686561066100ca000000b400000024686d74780474006a000000f8000000086c6f6361002e00140000012c000000066d6178700005000b000000d8000000200001000000010000f59c29445f0f3cf5000203e800000000b492f40000000000dc2fa65c00060000025802bc000000030002000000000000000100000400fe70000002580006ffff0258000100000000000000000000000000000002000100000002000b00020000000000000000000000000000000000000000000002580064021c000600000001000000030000000c00040020000000040004000100000041ffff00000041ffffffc000010000000000000014002e0000000200640000025802bc00030007000033112111252111216401f4fe3401a4fe5c02bcfd4428026c000200060000021d02900002000a00001333030113331323272307adc463fef8da60dd593eef42010b0140fdb50290fd70c8c800";
    HEX.as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let digit = |byte: u8| match byte {
                b'0'..=b'9' => byte - b'0',
                b'a'..=b'f' => byte - b'a' + 10,
                _ => unreachable!("static lowercase hexadecimal"),
            };
            digit(pair[0]) * 16 + digit(pair[1])
        })
        .collect()
}

#[test]
fn certified_layer_images_emit_x2_copper_and_plated_excellon_files() {
    let signal = NetId::new("SIGNAL").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("fab").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    });
    let front = TraceLayer(0);
    let back = TraceLayer(1);
    let layout = PcbLayout {
        id: BoardId::new("fab").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(0, 0), p(20, 0), p(20, 10), p(0, 10)].into(),
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
        routes: Vec::new(),
        vias: vec![PcbVia {
            id: ViaId::new("V1").unwrap(),
            net: signal,
            start_layer: front,
            end_layer: back,
            center: p(10, 5),
            land_diameter: Real::from(4),
            drill_diameter: Real::from(2),
            plating: Plating::Plated,
            mask: hypercircuit::ViaMaskIntent {
                front: hypercircuit::ViaMaskDisposition::Open {
                    margin: Real::one(),
                },
                back: hypercircuit::ViaMaskDisposition::Tented,
            },
        }],
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };

    let mut materialized = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    materialized.drills.push(DrillHit {
        source: "mounting-slot".into(),
        center: p(5, 5),
        shape: DrillShape::Slot {
            start: p(4, 5),
            end: p(6, 5),
            width: Real::one(),
        },
        plating: Plating::NonPlated,
    });
    materialized
        .zone_realizations
        .push(ZoneMaterializationEvidence {
            source: "zone:ground".into(),
            priority: 3,
            fill: "hatched".into(),
            connection: "thermal-relief".into(),
            clearance: "1/5".into(),
            cleared_foreign_features: 2,
            treated_same_net_lands: 4,
            thermal_bounding_box_projections: 4,
            applied_keepouts: 1,
            initial_islands: 3,
            retained_islands: 2,
            pruned_unconnected_islands: 1,
            pruned_below_area_islands: 0,
        });
    let package = FabricationPackage::from_materialization(&layout, &materialized).unwrap();
    assert_eq!(package.represented_copper_features, 2);
    assert_eq!(package.represented_process_features, 1);
    assert_eq!(package.represented_drills, 2);
    assert_eq!(package.files.len(), 8);
    assert!(
        package
            .files
            .iter()
            .filter(|file| {
                file.kind == FabricationFileKind::GerberX2
                    && file
                        .bytes
                        .windows(b"TF.FileFunction,Copper".len())
                        .any(|window| window == b"TF.FileFunction,Copper")
            })
            .count()
            == 2
    );
    assert!(package.files.iter().any(|file| {
        file.kind == FabricationFileKind::Excellon && file.bytes.starts_with(b"M48\n")
    }));
    assert!(package.files.iter().any(|file| {
        file.kind == FabricationFileKind::Ipc356
            && std::str::from_utf8(&file.bytes)
                .unwrap()
                .contains("327 /SIGNAL VIA V1 X10.000000Y5.000000 FEATURE=VIA")
    }));
    assert!(package.files.iter().any(|file| {
        file.name.ends_with("NPTH.drl")
            && file
                .bytes
                .windows(b"M15\nG01".len())
                .any(|window| window == b"M15\nG01")
    }));
    assert!(package.files.iter().any(|file| {
        file.name.ends_with("Profile.gbr")
            && file
                .bytes
                .windows(b"TF.FileFunction,Profile,NP".len())
                .any(|window| window == b"TF.FileFunction,Profile,NP")
    }));
    assert!(package.files.iter().any(|file| {
        file.name.ends_with("F_Mask.gbr")
            && file
                .bytes
                .windows(b"TF.FileFunction,Soldermask,Top".len())
                .any(|window| window == b"TF.FileFunction,Soldermask,Top")
    }));
    assert!(!package.production_omissions.iter().any(|omission| {
        matches!(
            omission,
            ProcessMaterializationOmission::ViaMaskIntentUnavailable { .. }
        )
    }));
    assert_eq!(package.manifest.files.len(), 7);
    assert_eq!(package.manifest.represented_test_points, 1);
    assert_eq!(package.manifest.test_points.len(), 1);
    assert!(
        package
            .manifest
            .files
            .iter()
            .all(|file| file.sha256.len() == 64)
    );
    assert!(package.verify_integrity().is_empty());
    assert!(package.files.iter().any(|file| {
        file.kind == FabricationFileKind::Manifest
            && file
                .bytes
                .starts_with(b"{\n  \"schema\": \"hypercircuit.fabrication-manifest\"")
    }));
    let manifest_file = package
        .files
        .iter()
        .find(|file| file.kind == FabricationFileKind::Manifest)
        .unwrap();
    let decoded = FabricationManifest::from_json(&manifest_file.bytes).unwrap();
    assert_eq!(decoded, package.manifest);
    assert_eq!(decoded.zone_realizations, materialized.zone_realizations);
    assert!(decoded.verify_files(&package.files).is_empty());
    let mut legacy_value = serde_json::to_value(&decoded).unwrap();
    let legacy = legacy_value.as_object_mut().unwrap();
    legacy.insert("version".into(), serde_json::json!(7));
    legacy.remove("represented_test_points");
    legacy.remove("test_points");
    legacy.remove("connectivity_omissions");
    let migrated = FabricationManifest::from_json(&serde_json::to_vec(&legacy_value).unwrap())
        .expect("version 7 manifests remain readable");
    assert_eq!(migrated.version, hypercircuit::FABRICATION_MANIFEST_VERSION);
    assert_eq!(migrated.represented_test_points, 0);
    assert!(migrated.test_points.is_empty());
    let mut version_eight_value = serde_json::to_value(&decoded).unwrap();
    version_eight_value
        .as_object_mut()
        .unwrap()
        .insert("version".into(), serde_json::json!(8));
    let migrated =
        FabricationManifest::from_json(&serde_json::to_vec(&version_eight_value).unwrap())
            .expect("version 8 manifests remain readable");
    assert_eq!(migrated.version, hypercircuit::FABRICATION_MANIFEST_VERSION);
    assert_eq!(migrated.files, decoded.files);

    #[cfg(feature = "drc")]
    {
        let audit = package.audit_cam_round_trip();
        assert!(
            audit.is_release_clean(),
            "CAM re-import issues: {:?}",
            audit.issues
        );
        assert_eq!(audit.manifest.as_ref(), Some(&package.manifest));
        assert_eq!(audit.gerbers.len(), 4);
        assert!(
            audit
                .gerbers
                .iter()
                .all(|evidence| evidence.geometry_nonempty
                    && !evidence.file_function.is_empty()
                    && evidence.coordinate_operations > 0
                    && evidence.aperture_definitions > 0)
        );
        assert_eq!(
            audit
                .excellons
                .iter()
                .map(|evidence| evidence.round_hits)
                .sum::<usize>(),
            1
        );
        assert_eq!(audit.ipc356.as_ref().unwrap().points, 1);
        assert_eq!(audit.ipc356.as_ref().unwrap().named_net_records, 1);
        assert_eq!(
            audit
                .excellons
                .iter()
                .map(|evidence| evidence.routed_slots)
                .sum::<usize>(),
            1
        );
        assert!(
            audit
                .excellons
                .iter()
                .any(|evidence| evidence.plated && evidence.round_hits == 1)
        );
        assert!(
            audit
                .excellons
                .iter()
                .any(|evidence| !evidence.plated && evidence.routed_slots == 1)
        );

        let mut files_with_wrong_drill_total = package.files.clone();
        let manifest_file = files_with_wrong_drill_total
            .iter_mut()
            .find(|file| file.kind == FabricationFileKind::Manifest)
            .unwrap();
        let mut altered_manifest = FabricationManifest::from_json(&manifest_file.bytes).unwrap();
        altered_manifest.represented_drills += 1;
        manifest_file.bytes = serde_json::to_vec_pretty(&altered_manifest).unwrap();
        manifest_file.bytes.push(b'\n');
        let altered_audit =
            hypercircuit::FabricationCamRoundTripReport::from_files(&files_with_wrong_drill_total);
        assert!(altered_audit.issues.iter().any(|issue| matches!(
            issue,
            hypercircuit::FabricationCamRoundTripIssue::DrillCount {
                expected: 3,
                parsed: 2
            }
        )));

        let files_without_manifest = package
            .files
            .iter()
            .filter(|file| file.kind != FabricationFileKind::Manifest)
            .cloned()
            .collect::<Vec<_>>();
        let missing_manifest_audit =
            hypercircuit::FabricationCamRoundTripReport::from_files(&files_without_manifest);
        assert_eq!(
            missing_manifest_audit.issues,
            vec![hypercircuit::FabricationCamRoundTripIssue::MissingManifest]
        );

        let mut damaged_connectivity = package.files.clone();
        damaged_connectivity
            .iter_mut()
            .find(|file| file.kind == FabricationFileKind::Ipc356)
            .unwrap()
            .bytes = b"327 /OTHER VIA V1 X10.000000Y5.000000 FEATURE=VIA\n".to_vec();
        let connectivity_audit =
            hypercircuit::FabricationCamRoundTripReport::from_files(&damaged_connectivity);
        assert!(connectivity_audit.issues.iter().any(|issue| matches!(
            issue,
            hypercircuit::FabricationCamRoundTripIssue::Ipc356 { .. }
        )));
    }

    let mut damaged = package.clone();
    damaged
        .files
        .iter_mut()
        .find(|file| file.kind == FabricationFileKind::Excellon)
        .unwrap()
        .bytes
        .push(b'!');
    assert!(damaged.verify_integrity().iter().any(|issue| matches!(
        issue,
        FabricationIntegrityIssue::DigestMismatch(_)
            | FabricationIntegrityIssue::ByteLengthMismatch(_)
    )));
}

#[test]
fn pad_and_artwork_intent_emit_unit_aware_mask_paste_and_legend_images() {
    let signal = NetId::new("SIGNAL").unwrap();
    let model = DeviceModelId::new("resistor").unwrap();
    let instance = CircuitInstanceId::new("R1").unwrap();
    let pin = PinRef::new("1").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("process").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Resistor,
        pins: vec![DevicePin {
            pin: pin.clone(),
            kind: PinElectricalKind::Passive,
            optional: false,
        }],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: instance.clone(),
        component: ComponentId::new("R1").unwrap(),
        part: None,
        model,
        pins: vec![PinBinding {
            pin: pin.clone(),
            net: signal,
        }],
        parameters: Vec::new(),
    });
    let front = TraceLayer(0);
    let back = TraceLayer(1);
    let pattern = LandPatternId::new("R_0603").unwrap();
    let layout = PcbLayout {
        id: BoardId::new("process").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(0, 0), p(1, 0), p(1, 1), p(0, 1)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![
                StackupLayer {
                    name: "F.Cu".into(),
                    kind: StackupLayerKind::Conductor(front),
                    thickness: Real::one(),
                    material: None,
                },
                StackupLayer {
                    name: "B.Cu".into(),
                    kind: StackupLayerKind::Conductor(back),
                    thickness: Real::one(),
                    material: None,
                },
            ],
        },
        land_patterns: vec![LandPattern {
            id: pattern.clone(),
            pads: vec![LandPatternPad {
                id: PadId::new("1").unwrap(),
                center: Point2::new(Real::zero(), Real::zero()),
                rotation_degrees: Real::zero(),
                copper_layers: vec![front],
                shape: PadShape::Rectangle {
                    width: Real::one(),
                    height: Real::one(),
                },
                drill: None,
                plating: Plating::Unspecified,
                solder_mask_margin: Some(
                    (Real::one() / Real::from(10)).expect("nonzero denominator"),
                ),
                paste_margin: Some((-Real::one() / Real::from(10)).expect("nonzero denominator")),
            }],
            pin_map: vec![PadPinMap {
                pin,
                pad: PadId::new("1").unwrap(),
            }],
            graphics: vec![
                LandPatternGraphic {
                    id: LandPatternGraphicId::new("pin-one-mark").unwrap(),
                    layer: LayerRole::FrontSilkscreen,
                    stroke_width: None,
                    primitive: LandPatternGraphicPrimitive::Polygon {
                        vertices: vec![p(-1, -1), p(1, -1), p(0, 1)],
                        filled: true,
                    },
                },
                LandPatternGraphic {
                    id: LandPatternGraphicId::new("reference-text").unwrap(),
                    layer: LayerRole::FrontSilkscreen,
                    stroke_width: None,
                    primitive: LandPatternGraphicPrimitive::Text {
                        text: "A".into(),
                        position: p(-3, 0),
                        height: Real::from(2),
                        rotation_degrees: Real::zero(),
                    },
                },
                LandPatternGraphic {
                    id: LandPatternGraphicId::new("copper-heat-spreader").unwrap(),
                    layer: LayerRole::Copper(front),
                    stroke_width: None,
                    primitive: LandPatternGraphicPrimitive::Polygon {
                        vertices: vec![p(-1, 0), p(1, 0), p(1, 1), p(-1, 1)],
                        filled: true,
                    },
                },
            ],
            body: None,
            models: Vec::new(),
        }],
        placements: vec![PcbPlacement {
            instance,
            land_pattern: pattern,
            position: Point2::new(
                (Real::one() / Real::from(2)).expect("nonzero denominator"),
                (Real::one() / Real::from(2)).expect("nonzero denominator"),
            ),
            rotation_degrees: Real::zero(),
            side: BoardSide::Back,
        }],
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };

    let unresolved_text = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    assert!(
        unresolved_text
            .process_omissions
            .iter()
            .any(|omission| matches!(
                omission,
                ProcessMaterializationOmission::TextArtwork { source }
                    if source.ends_with(":reference-text")
            ))
    );

    let materialized = layout
        .materialize(
            &circuit,
            MaterializationOptions {
                production_text: Some(ProductionTextPolicy {
                    font_name: "ttf-parser-demo".into(),
                    font_data: demo_font(),
                }),
                ..MaterializationOptions::default()
            },
        )
        .unwrap();
    assert!(
        materialized
            .copper_features
            .iter()
            .any(|feature| feature.kind == CopperFeatureKind::Artwork && feature.net.is_none())
    );
    assert_eq!(materialized.process_features.len(), 4);
    assert!(materialized.process_layers.iter().any(|image| {
        image.role == ProcessLayerRole::BackSolderMask && image.source_feature_count == 1
    }));
    assert!(materialized.process_layers.iter().any(|image| {
        image.role == ProcessLayerRole::BackPaste && image.source_feature_count == 1
    }));
    assert!(materialized.process_layers.iter().any(|image| {
        image.role == ProcessLayerRole::BackSilkscreen && image.source_feature_count == 2
    }));
    let text_evidence = materialized.production_text.as_ref().unwrap();
    assert_eq!(text_evidence.font_name, "ttf-parser-demo");
    assert_eq!(text_evidence.sha256.len(), 64);
    assert!(!materialized.process_omissions.iter().any(|omission| {
        matches!(
            omission,
            ProcessMaterializationOmission::TextArtwork { .. }
                | ProcessMaterializationOmission::TextFontRejected { .. }
        )
    }));
    assert!(
        materialized
            .process_omissions
            .contains(&ProcessMaterializationOmission::SilkscreenNotClippedToMask)
    );

    let package = FabricationPackage::from_materialization_with_options(
        &layout,
        &materialized,
        FabricationExportOptions {
            source_length_unit: FabricationLengthUnit::Inch,
            ..FabricationExportOptions::default()
        },
    )
    .unwrap();
    assert_eq!(package.represented_process_features, 4);
    assert_eq!(
        package.manifest.production_text,
        materialized.production_text
    );
    for function in ["Soldermask,Bot", "Paste,Bot", "Legend,Bot"] {
        assert!(package.files.iter().any(|file| {
            file.kind == FabricationFileKind::GerberX2
                && file
                    .bytes
                    .windows(function.len())
                    .any(|window| window == function.as_bytes())
        }));
    }
    let profile = package
        .files
        .iter()
        .find(|file| file.name.ends_with("Profile.gbr"))
        .unwrap();
    assert!(
        profile
            .bytes
            .windows(b"X0025400000Y0000000000D01*".len())
            .any(|window| window == b"X0025400000Y0000000000D01*")
    );
    assert_eq!(
        package.manifest.source_length_unit,
        FabricationLengthUnit::Inch
    );
    assert_eq!(
        package.manifest.output_length_unit,
        FabricationLengthUnit::Millimeter
    );
    assert_eq!(package.manifest.represented_process_features, 4);
    assert_eq!(package.production_omissions.len(), 1);
    assert!(package.verify_integrity().is_empty());

    let rejected = layout
        .materialize(
            &circuit,
            MaterializationOptions {
                production_text: Some(ProductionTextPolicy {
                    font_name: "invalid-font".into(),
                    font_data: b"not a font".to_vec(),
                }),
                ..MaterializationOptions::default()
            },
        )
        .unwrap();
    assert!(rejected.process_omissions.iter().any(|omission| matches!(
        omission,
        ProcessMaterializationOmission::TextFontRejected { source }
            if source.ends_with(":reference-text")
    )));
}
