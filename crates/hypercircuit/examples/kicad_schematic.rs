use std::path::PathBuf;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitPort, ComponentId,
    DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, KiCadSchematicExportOptions,
    KiCadSchematicImportReport, Net, NetId, PinBinding, PinElectricalKind, PinRef, PortDirection,
    PortId, SchematicAutoLayoutPolicy, TransientPolicy,
};

fn resistor(id: &str, first: &NetId, second: &NetId) -> CircuitInstance {
    CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: None,
        model: DeviceModelId::new("resistor").unwrap(),
        pins: vec![
            PinBinding {
                pin: PinRef::new("1").unwrap(),
                net: first.clone(),
            },
            PinBinding {
                pin: PinRef::new("2").unwrap(),
                net: second.clone(),
            },
        ],
        parameters: Vec::new(),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let output = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("divider.kicad_sch"));
    let supply = NetId::new("VCC")?;
    let output_net = NetId::new("OUT")?;
    let ground = NetId::new("GND")?;
    let circuit = Circuit::new(
        CircuitId::new("native-kicad-divider")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: supply.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: output_net.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_port(CircuitPort {
        id: PortId::new("VIN")?,
        net: supply.clone(),
        direction: PortDirection::Input,
        optional: false,
    })
    .with_port(CircuitPort {
        id: PortId::new("VOUT")?,
        net: output_net.clone(),
        direction: PortDirection::Output,
        optional: false,
    })
    .with_device_model(DeviceModel {
        id: DeviceModelId::new("resistor")?,
        kind: DeviceModelKind::Resistor,
        pins: vec![
            DevicePin {
                pin: PinRef::new("1")?,
                kind: PinElectricalKind::Passive,
                optional: false,
            },
            DevicePin {
                pin: PinRef::new("2")?,
                kind: PinElectricalKind::Passive,
                optional: false,
            },
        ],
        parameters: Vec::new(),
    })
    .with_instance(resistor("R1", &supply, &output_net))
    .with_instance(resistor("R2", &output_net, &ground));

    let generated = circuit.auto_schematic(SchematicAutoLayoutPolicy::default())?;
    let exported = generated
        .layout
        .export_kicad_schematic(&circuit, KiCadSchematicExportOptions::default())?;
    std::fs::write(&output, &exported.schematic)?;
    let reimported = KiCadSchematicImportReport::from_str(&exported.schematic, &circuit)?;

    println!(
        "wrote {}: {} symbols, {} circuit wires -> {} native segments, {} imported drawing wires",
        output.display(),
        exported.symbols.len(),
        exported.wires.len(),
        exported
            .wires
            .iter()
            .map(|wire| wire.native_uuids.len())
            .sum::<usize>(),
        reimported.layout.wires.len()
    );
    Ok(())
}
