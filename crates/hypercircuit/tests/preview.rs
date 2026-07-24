#![cfg(feature = "layout")]

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, BoardSide, Circuit, CircuitId, CircuitInstance,
    CircuitInstanceId, ComponentId, CopperZone, DeviceModel, DeviceModelId, DeviceModelKind,
    DevicePin, LandPattern, LandPatternId, LandPatternPad, Net, NetId, PadId, PadPinMap, PadShape,
    PcbDesignRules, PcbLayout, PcbPlacement, PcbRoute, PcbStackup, PcbSvgOptions, PcbVia,
    PinBinding, PinElectricalKind, PinRef, Plating, Real, RouteId, StackupLayer, StackupLayerKind,
    TransientPolicy, ViaId, ZoneId,
};
use hyperlattice::Point2;
use hyperpath::{LinePathSegment, TraceLayer};

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn fixture() -> (Circuit, PcbLayout) {
    let net = NetId::new("SIGNAL").unwrap();
    let model = DeviceModelId::new("preview-part").unwrap();
    let instance = CircuitInstanceId::new("U1").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("preview").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: net.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Custom("preview".into()),
        pins: vec![DevicePin {
            pin: PinRef::new("1").unwrap(),
            kind: PinElectricalKind::Passive,
            optional: false,
        }],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: instance.clone(),
        component: ComponentId::new("U1").unwrap(),
        part: None,
        model,
        pins: vec![PinBinding {
            pin: PinRef::new("1").unwrap(),
            net: net.clone(),
        }],
        parameters: Vec::new(),
    });
    let pattern = LandPatternId::new("QFN-preview").unwrap();
    let layout = PcbLayout {
        id: BoardId::new("preview-board").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(0, 0), p(40, 0), p(40, 30), p(0, 30)].into(),
            cutouts: vec![vec![p(2, 2), p(4, 2), p(4, 4), p(2, 4)].into()],
        },
        stackup: PcbStackup {
            layers: vec![
                StackupLayer {
                    name: "F.Cu".into(),
                    kind: StackupLayerKind::Conductor(TraceLayer(0)),
                    thickness: Real::one(),
                    material: None,
                },
                StackupLayer {
                    name: "B.Cu".into(),
                    kind: StackupLayerKind::Conductor(TraceLayer(1)),
                    thickness: Real::one(),
                    material: None,
                },
            ],
        },
        land_patterns: vec![LandPattern {
            id: pattern.clone(),
            pads: vec![LandPatternPad {
                id: PadId::new("1").unwrap(),
                center: p(2, 0),
                rotation_degrees: Real::from(30),
                copper_layers: vec![TraceLayer(0)],
                shape: PadShape::RoundedRectangle {
                    width: Real::from(4),
                    height: Real::from(2),
                    corner_radius: (Real::one() / Real::from(2)).unwrap(),
                },
                drill: None,
                plating: Plating::Unspecified,
                solder_mask_margin: None,
                paste_margin: None,
            }],
            pin_map: vec![PadPinMap {
                pin: PinRef::new("1").unwrap(),
                pad: PadId::new("1").unwrap(),
            }],
            graphics: Vec::new(),
            body: None,
            models: Vec::new(),
        }],
        placements: vec![PcbPlacement {
            instance,
            land_pattern: pattern,
            position: p(8, 8),
            rotation_degrees: Real::from(90),
            side: BoardSide::Front,
        }],
        placement_constraints: Vec::new(),
        routes: vec![PcbRoute {
            id: RouteId::new("front-route").unwrap(),
            net: net.clone(),
            layer: TraceLayer(0),
            width: Real::from(2),
            segments: vec![LinePathSegment::new(p(5, 5), p(20, 5)).into()],
        }],
        vias: vec![PcbVia {
            id: ViaId::new("through-via").unwrap(),
            net: net.clone(),
            start_layer: TraceLayer(0),
            end_layer: TraceLayer(1),
            center: p(20, 5),
            land_diameter: Real::from(4),
            drill_diameter: Real::from(2),
            plating: Plating::Plated,
            mask: hypercircuit::ViaMaskIntent::default(),
        }],
        zones: vec![CopperZone {
            id: ZoneId::new("back-zone").unwrap(),
            net,
            layer: TraceLayer(1),
            boundary: vec![p(10, 10), p(30, 10), p(30, 20), p(10, 20)],
            clearance: Real::zero(),
            fill: hypercircuit::CopperZoneFill::Solid,
            connection: hypercircuit::CopperZoneConnection::Solid,
            islands: hypercircuit::CopperZoneIslandPolicy::retain_all(),
            stitching: None,
            priority: 0,
        }],
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };
    (circuit, layout)
}

#[test]
fn native_pcb_svg_retains_source_and_net_identity_with_projection_audit() {
    let (circuit, layout) = fixture();
    let report = layout.to_svg(&circuit, PcbSvgOptions::default()).unwrap();
    assert!(report.svg.starts_with("<svg"));
    assert!(report.svg.contains("data-source=\"front-route\""));
    assert!(report.svg.contains("data-source=\"through-via\""));
    assert!(report.svg.contains("data-source=\"back-zone\""));
    assert!(report.svg.contains("data-net=\"SIGNAL\""));
    assert!(report.svg.contains("data-instance=\"U1\""));
    assert!(report.svg.contains("data-pad=\"1\""));
    assert!(report.svg.contains("rotate(120 "));
    assert!(!report.projections.is_empty());
}

#[test]
fn native_pcb_svg_filters_copper_by_layer_but_keeps_spanning_vias() {
    let (circuit, layout) = fixture();
    let report = layout
        .to_svg(
            &circuit,
            PcbSvgOptions {
                layer: Some(TraceLayer(1)),
                ..PcbSvgOptions::default()
            },
        )
        .unwrap();
    assert!(!report.svg.contains("front-route"));
    assert!(report.svg.contains("back-zone"));
    assert!(report.svg.contains("through-via"));
    assert!(!report.svg.contains("data-pad=\"1\""));
}
