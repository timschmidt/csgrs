//! Compose two instances of one reusable circuit/layout module.

use std::error::Error;

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, BoardSide, Circuit, CircuitId, CircuitInstance,
    CircuitInstanceId, CircuitLibrary, CircuitPort, ComponentId, DeviceModel, DeviceModelId,
    DeviceModelKind, LandPattern, LandPatternId, LayoutAssembly, LayoutModule, LayoutModuleId,
    LayoutModuleInstance, LayoutTransform, Net, NetId, PcbDesignRules, PcbLayout, PcbPlacement,
    PcbRoute, PcbRouteSegment, PcbStackup, PortDirection, PortId, Real, RouteId, StackupLayer,
    StackupLayerKind, SubcircuitInstance, SubcircuitInstanceId, SubcircuitPortBinding,
    TransientPolicy,
};
use hyperlattice::Point2;
use hyperpath::{LinePathSegment, TraceLayer};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn main() -> Result<(), Box<dyn Error>> {
    let signal = NetId::new("SIGNAL")?;
    let module_model = DeviceModelId::new("module-part")?;
    let module_circuit = Circuit::new(
        CircuitId::new("filter")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    })
    .with_port(CircuitPort {
        id: PortId::new("signal")?,
        net: signal.clone(),
        direction: PortDirection::Passive,
        optional: false,
    })
    .with_device_model(DeviceModel {
        id: module_model.clone(),
        kind: DeviceModelKind::Custom("filter-part".into()),
        pins: Vec::new(),
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("U1")?,
        component: ComponentId::new("U1")?,
        part: None,
        model: module_model,
        pins: Vec::new(),
        parameters: Vec::new(),
    });
    let root_signal = NetId::new("ROOT_SIGNAL")?;
    let child = |id: &str| -> Result<SubcircuitInstance, Box<dyn Error>> {
        Ok(SubcircuitInstance {
            id: SubcircuitInstanceId::new(id)?,
            circuit: CircuitId::new("filter")?,
            ports: vec![SubcircuitPortBinding {
                port: PortId::new("signal")?,
                net: root_signal.clone(),
            }],
            parameter_overrides: Vec::new(),
        })
    };
    let root = Circuit::new(
        CircuitId::new("module-board")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: root_signal.clone(),
        is_ground: false,
    })
    .with_subcircuit(child("left")?)
    .with_subcircuit(child("right")?);
    let circuits = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, module_circuit],
    };
    let pattern = LandPatternId::new("FILTER_PATTERN")?;
    let module = LayoutModule {
        id: LayoutModuleId::new("filter-layout")?,
        circuit: CircuitId::new("filter")?,
        land_patterns: vec![LandPattern {
            id: pattern.clone(),
            pads: Vec::new(),
            pin_map: Vec::new(),
            graphics: Vec::new(),
            body: None,
            models: Vec::new(),
        }],
        placements: vec![PcbPlacement {
            instance: CircuitInstanceId::new("U1")?,
            land_pattern: pattern,
            position: point(1, 1),
            rotation_degrees: Real::zero(),
            side: BoardSide::Front,
        }],
        placement_constraints: Vec::new(),
        placement_groups: Vec::new(),
        routes: vec![PcbRoute {
            id: RouteId::new("signal-route")?,
            net: signal,
            layer: TraceLayer(0),
            width: Real::one(),
            segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
                point(0, 0),
                point(2, 0),
            ))],
        }],
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };
    let board = PcbLayout {
        id: BoardId::new("module-board")?,
        outline: BoardOutline {
            exterior: vec![point(0, 0), point(30, 0), point(30, 10), point(0, 10)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(TraceLayer(0)),
                thickness: Real::one(),
                material: Some("copper".into()),
            }],
        },
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };
    let report = LayoutAssembly {
        board,
        modules: vec![module],
        instances: vec![
            LayoutModuleInstance {
                hierarchy_path: vec![SubcircuitInstanceId::new("left")?],
                module: LayoutModuleId::new("filter-layout")?,
                transform: LayoutTransform {
                    position: point(5, 2),
                    ..LayoutTransform::default()
                },
            },
            LayoutModuleInstance {
                hierarchy_path: vec![SubcircuitInstanceId::new("right")?],
                module: LayoutModuleId::new("filter-layout")?,
                transform: LayoutTransform {
                    position: point(20, 2),
                    side: BoardSide::Back,
                    ..LayoutTransform::default()
                },
            },
        ],
    }
    .compose(&circuits)?;
    println!(
        "composed {} modules into {} placements and {} routes",
        report.modules.len(),
        report.layout.placements.len(),
        report.layout.routes.len()
    );
    for placement in &report.layout.placements {
        println!(
            "{} at ({}, {}) {:?}",
            placement.instance.as_str(),
            placement.position.x,
            placement.position.y,
            placement.side
        );
    }
    Ok(())
}
