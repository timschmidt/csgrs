use std::error::Error;
use std::path::PathBuf;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, ComponentId, DeviceModel,
    DeviceModelId, DeviceModelKind, DevicePin, KiCadSchematicExportOptions, Net, NetId, PinBinding,
    PinElectricalKind, PinRef, Real, SchematicGraphic, SchematicGraphicFill, SchematicLayout,
    SchematicPinPlacement, SchematicPinSide, SchematicPoint, SchematicSvgOptions, SchematicSymbol,
    SchematicSymbolDefinition, SchematicSymbolDefinitionId, SchematicSymbolId, SchematicSymbolUnit,
    TransientPolicy,
};

fn point(x: i64, y: i64) -> SchematicPoint {
    SchematicPoint::new(Real::from(x), Real::from(y))
}

fn main() -> Result<(), Box<dyn Error>> {
    let output = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("multipart.kicad_sch"));
    let svg_output = output.with_extension("svg");
    let model = DeviceModelId::new("dual-buffer")?;
    let instance = CircuitInstanceId::new("U1")?;
    let pin_names = ["A_IN", "A_OUT", "B_IN", "B_OUT"];
    let nets = pin_names
        .iter()
        .map(|name| NetId::new(*name))
        .collect::<Result<Vec<_>, _>>()?;
    let circuit = pin_names
        .iter()
        .zip(&nets)
        .fold(
            Circuit::new(
                CircuitId::new("multipart-example")?,
                TransientPolicy::Static,
                AdapterKind::Dc,
            ),
            |circuit, (_, net)| {
                circuit.with_net(Net {
                    id: net.clone(),
                    is_ground: false,
                })
            },
        )
        .with_device_model(DeviceModel {
            id: model.clone(),
            kind: DeviceModelKind::Custom("two independent buffers".into()),
            pins: pin_names
                .iter()
                .enumerate()
                .map(|(index, name)| {
                    Ok(DevicePin {
                        pin: PinRef::new(*name)?,
                        kind: if index.is_multiple_of(2) {
                            PinElectricalKind::Input
                        } else {
                            PinElectricalKind::Output
                        },
                        optional: false,
                    })
                })
                .collect::<Result<Vec<_>, hypercircuit::CircuitError>>()?,
            parameters: Vec::new(),
        })
        .with_instance(CircuitInstance {
            id: instance.clone(),
            component: ComponentId::new("U1")?,
            part: None,
            model: model.clone(),
            pins: pin_names
                .iter()
                .zip(&nets)
                .map(|(name, net)| {
                    Ok(PinBinding {
                        pin: PinRef::new(*name)?,
                        net: net.clone(),
                    })
                })
                .collect::<Result<Vec<_>, hypercircuit::CircuitError>>()?,
            parameters: Vec::new(),
        });

    let unit = |number: u16, input: &str, output: &str, label: &str| {
        Ok::<_, hypercircuit::CircuitError>(SchematicSymbolUnit {
            unit: number,
            body_width: Real::from(16),
            body_height: Real::from(10),
            pins: vec![
                SchematicPinPlacement {
                    pin: PinRef::new(input)?,
                    position: point(-10, 0),
                    side: SchematicPinSide::Left,
                },
                SchematicPinPlacement {
                    pin: PinRef::new(output)?,
                    position: point(10, 0),
                    side: SchematicPinSide::Right,
                },
            ],
            graphics: vec![
                SchematicGraphic::Rectangle {
                    start: point(-8, -5),
                    end: point(8, 5),
                    stroke_width: Real::one(),
                    fill: SchematicGraphicFill::Background,
                },
                SchematicGraphic::Polyline {
                    points: vec![point(-4, -3), point(4, 0), point(-4, 3)],
                    closed: true,
                    stroke_width: Real::one(),
                    fill: SchematicGraphicFill::None,
                },
                SchematicGraphic::Text {
                    position: point(0, -3),
                    text: label.into(),
                    size: Real::from(2),
                    quarter_turns: 0,
                },
            ],
        })
    };
    let definition = SchematicSymbolDefinitionId::new("dual-buffer-symbol")?;
    let layout = SchematicLayout {
        symbol_definitions: vec![SchematicSymbolDefinition {
            id: definition.clone(),
            model,
            name: "Dual buffer".into(),
            units: vec![
                unit(1, "A_IN", "A_OUT", "A")?,
                unit(2, "B_IN", "B_OUT", "B")?,
            ],
        }],
        symbols: vec![
            SchematicSymbol {
                id: SchematicSymbolId::new("U1:A")?,
                instance: instance.clone(),
                definition: definition.clone(),
                unit: 1,
                position: point(20, 20),
                quarter_turns: 0,
            },
            SchematicSymbol {
                id: SchematicSymbolId::new("U1:B")?,
                instance,
                definition,
                unit: 2,
                position: point(55, 20),
                quarter_turns: 0,
            },
        ],
        ..SchematicLayout::default()
    };
    let kicad = layout.export_kicad_schematic(&circuit, KiCadSchematicExportOptions::default())?;
    std::fs::write(&output, kicad.schematic)?;
    let svg = layout.to_svg(&circuit, SchematicSvgOptions::default())?;
    std::fs::write(&svg_output, svg.svg)?;
    println!(
        "wrote {} and {} from one shared two-unit definition",
        output.display(),
        svg_output.display()
    );
    Ok(())
}
