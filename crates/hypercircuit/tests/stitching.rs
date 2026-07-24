#![cfg(all(
    feature = "geometry",
    feature = "interchange",
    feature = "lceda",
    feature = "drc"
))]

use hypercircuit::{
    AdapterKind, BoardContour, BoardId, BoardOutline, Circuit, CircuitId, CopperZone,
    CopperZoneIslandPolicy, CopperZoneStitchingPolicy, FabricationPackage, KeepoutId, KeepoutScope,
    LcedaProExportOptions, LcedaProExportReport, MaterializationOptions, Net, NetId,
    PcbDesignRules, PcbKeepout, PcbLayout, PcbStackup, PcbVia, Plating, Real, SemanticDocument,
    StackupLayer, StackupLayerKind, TransientPolicy, ViaId, ViaMaskIntent, ZoneId,
};
use hyperlattice::Point2;
use hyperpath::{ArcDirection, ExplicitCircularArc, LinePathSegment, TraceLayer};

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn fixture(maximum_vias: usize) -> (Circuit, PcbLayout) {
    let ground = NetId::new("GND").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("stitching").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    });
    let front = TraceLayer(0);
    let back = TraceLayer(1);
    let layout = PcbLayout {
        id: BoardId::new("stitching").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(0, 0), p(20, 0), p(20, 20), p(0, 20)].into(),
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
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: vec![PcbVia {
            id: ViaId::new("authored-via").unwrap(),
            net: ground.clone(),
            start_layer: front,
            end_layer: back,
            center: p(14, 14),
            land_diameter: Real::from(2),
            drill_diameter: Real::one(),
            plating: Plating::Plated,
            mask: ViaMaskIntent::tented(),
        }],
        zones: vec![CopperZone {
            id: ZoneId::new("ground-zone").unwrap(),
            net: ground,
            layer: front,
            boundary: vec![p(0, 0), p(20, 0), p(20, 20), p(0, 20)],
            clearance: Real::zero(),
            fill: hypercircuit::CopperZoneFill::Solid,
            connection: hypercircuit::CopperZoneConnection::Solid,
            islands: CopperZoneIslandPolicy::retain_all(),
            stitching: Some(CopperZoneStitchingPolicy {
                pitch: Real::from(6),
                edge_clearance: Real::one(),
                start_layer: front,
                end_layer: back,
                land_diameter: Real::from(2),
                drill_diameter: Real::one(),
                mask: ViaMaskIntent::tented(),
                maximum_vias,
            }),
            priority: 0,
        }],
        keepouts: vec![PcbKeepout {
            id: KeepoutId::new("via-reserve").unwrap(),
            boundary: vec![p(7, 7), p(9, 7), p(9, 9), p(7, 9)],
            scope: KeepoutScope::Vias,
        }],
        rules: PcbDesignRules::default(),
    };
    (circuit, layout)
}

#[test]
fn deterministic_stitching_vias_feed_every_release_boundary() {
    let (circuit, layout) = fixture(100);
    assert!(layout.validate(&circuit).is_valid());

    let stitching = layout.realize_stitching_vias();
    assert!(stitching.is_complete());
    assert_eq!(stitching.vias.len(), 7);
    assert_eq!(stitching.vias[0].id.as_str(), "ground-zone-stitch-0-0");
    assert_eq!(stitching.evidence[0].candidates, 9);
    assert_eq!(stitching.evidence[0].accepted, 7);
    assert_eq!(stitching.evidence[0].rejected.keepout, 1);
    assert_eq!(stitching.evidence[0].rejected.via_collision, 1);

    let routing = hypercircuit::RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    assert_eq!(routing.problem.existing.vias().len(), 8);
    assert_eq!(routing.problem.existing_vias.len(), 8);
    assert_eq!(routing.stitching_realizations, stitching.evidence);

    let preview = layout
        .to_svg(&circuit, hypercircuit::PcbSvgOptions::default())
        .unwrap();
    assert!(preview.svg.contains("ground-zone-stitch-0-0"));

    let kicad = layout
        .export_kicad(&circuit, hypercircuit::KiCadExportOptions::default())
        .unwrap();
    assert_eq!(kicad.board.matches("\n  (via ").count(), 8);
    assert!(!kicad.omissions.iter().any(|omission| matches!(
        omission,
        hypercircuit::KiCadExportOmission::ZoneStitchingIncomplete { .. }
    )));
    assert!(kicad.omissions.iter().any(|omission| matches!(
        omission,
        hypercircuit::KiCadExportOmission::ZoneStitchingPolicyLowered {
            zone,
            generated_vias: 7
        } if zone == "ground-zone"
    )));

    let lceda = LcedaProExportReport::from_design(
        &circuit,
        None,
        Some(&layout),
        LcedaProExportOptions::millimeters(),
    )
    .unwrap();
    assert!(lceda.record_stream.contains("\"hypercircuitStitching\""));
    assert_eq!(lceda.record_stream.matches("\"type\":\"VIA\"").count(), 8);

    let semantic = SemanticDocument::new(circuit.clone(), None)
        .unwrap()
        .with_pcb(layout.clone())
        .unwrap();
    assert_eq!(
        SemanticDocument::from_json(&semantic.to_json_pretty().unwrap()).unwrap(),
        semantic
    );

    let materialized = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    assert_eq!(materialized.drills.len(), 8);
    assert_eq!(materialized.stitching_realizations, stitching.evidence);
    assert!(materialized.copper_features.iter().any(|feature| {
        feature.source == "via:ground-zone-stitch-0-0"
            && feature.kind == hypercircuit::CopperFeatureKind::Via
    }));
    let handoff = hypercircuit::HyperDrcHandoff::from_materialization(&layout, &materialized);
    assert_eq!(
        handoff
            .board
            .copper
            .iter()
            .filter(|feature| feature.kind == hyperdrc::kicad::CopperKind::Via)
            .count(),
        16
    );
    let fabrication = FabricationPackage::from_materialization(&layout, &materialized).unwrap();
    assert!(fabrication.verify_integrity().is_empty());
    assert_eq!(
        fabrication.manifest.stitching_realizations,
        stitching.evidence
    );
}

#[test]
fn stitching_uses_exact_arc_board_clearance_without_indeterminate_candidates() {
    let (circuit, mut layout) = fixture(100);
    layout.outline.exterior = BoardContour::from_segments(vec![
        LinePathSegment::new(p(0, 0), p(20, 0)).into(),
        LinePathSegment::new(p(20, 0), p(20, 20)).into(),
        ExplicitCircularArc::new(
            p(10, 20),
            Real::from(10),
            p(20, 20),
            p(0, 20),
            ArcDirection::Ccw,
        )
        .unwrap()
        .into(),
        LinePathSegment::new(p(0, 20), p(0, 0)).into(),
    ]);
    assert!(layout.validate(&circuit).is_valid());
    let report = layout.realize_stitching_vias();
    assert!(report.is_complete(), "{:?}", report.evidence);
    assert_eq!(report.evidence[0].rejected.indeterminate, 0);
    assert!(!report.vias.is_empty());
}

#[test]
fn stitching_cap_is_explicit_in_reports_and_exports() {
    let (circuit, layout) = fixture(2);
    let stitching = layout.realize_stitching_vias();
    assert_eq!(stitching.vias.len(), 2);
    assert!(stitching.evidence[0].truncated);
    assert!(!stitching.is_complete());

    let kicad = layout
        .export_kicad(&circuit, hypercircuit::KiCadExportOptions::default())
        .unwrap();
    assert!(kicad.omissions.iter().any(|omission| matches!(
        omission,
        hypercircuit::KiCadExportOmission::ZoneStitchingIncomplete { zone }
            if zone == "ground-zone"
    )));
}
