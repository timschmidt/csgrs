//! Instantiate one exact circuit module with two different parameter overrides.

use std::error::Error;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitLibrary,
    CircuitModuleParameter, CircuitModuleParameterOverride, CircuitModuleParameterTarget,
    CircuitParameter, CircuitPort, ComponentId, DeviceModel, DeviceModelId, DeviceModelKind,
    DevicePin, Net, NetId, PinBinding, PinElectricalKind, PinRef, PortDirection, PortId, Real,
    SubcircuitInstance, SubcircuitInstanceId, SubcircuitPortBinding, TransientPolicy,
};

fn parameter(name: &str, value: i64, unit: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value: Real::from(value),
        unit: unit.into(),
        source: "module-default".into(),
    }
}

fn resistor_module() -> Result<Circuit, Box<dyn Error>> {
    let output = NetId::new("OUT")?;
    let reference = NetId::new("REF")?;
    let model = DeviceModelId::new("resistor")?;
    let pins = ["+", "-"]
        .into_iter()
        .map(|name| {
            Ok(DevicePin {
                pin: PinRef::new(name)?,
                kind: PinElectricalKind::Passive,
                optional: false,
            })
        })
        .collect::<Result<Vec<_>, Box<dyn Error>>>()?;
    Ok(Circuit::new(
        CircuitId::new("resistor-module")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: reference.clone(),
        is_ground: true,
    })
    .with_port(CircuitPort {
        id: PortId::new("out")?,
        net: output.clone(),
        direction: PortDirection::Passive,
        optional: false,
    })
    .with_port(CircuitPort {
        id: PortId::new("ref")?,
        net: reference.clone(),
        direction: PortDirection::Ground,
        optional: false,
    })
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Resistor,
        pins,
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("R1")?,
        component: ComponentId::new("R1")?,
        part: None,
        model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("+")?,
                net: output,
            },
            PinBinding {
                pin: PinRef::new("-")?,
                net: reference,
            },
        ],
        parameters: vec![parameter("resistance", 10, "ohm")],
    })
    .with_module_parameter(CircuitModuleParameter {
        name: "resistance".into(),
        default: Real::from(10),
        unit: "ohm".into(),
        source: "module-default".into(),
        targets: vec![CircuitModuleParameterTarget::InstanceParameter {
            instance: CircuitInstanceId::new("R1")?,
            parameter: "resistance".into(),
        }],
    }))
}

fn module_instance(
    id: &str,
    output: &NetId,
    ground: &NetId,
    resistance: i64,
) -> Result<SubcircuitInstance, Box<dyn Error>> {
    Ok(SubcircuitInstance {
        id: SubcircuitInstanceId::new(id)?,
        circuit: CircuitId::new("resistor-module")?,
        ports: vec![
            SubcircuitPortBinding {
                port: PortId::new("out")?,
                net: output.clone(),
            },
            SubcircuitPortBinding {
                port: PortId::new("ref")?,
                net: ground.clone(),
            },
        ],
        parameter_overrides: vec![CircuitModuleParameterOverride {
            parameter: "resistance".into(),
            value: Real::from(resistance),
            source: format!("root.{id}.resistance"),
        }],
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let output = NetId::new("OUT")?;
    let ground = NetId::new("GND")?;
    let root = Circuit::new(
        CircuitId::new("parameterized-root")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_subcircuit(module_instance("load-a", &output, &ground, 2)?)
    .with_subcircuit(module_instance("load-b", &output, &ground, 4)?);
    let flattened = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, resistor_module()?],
    }
    .flatten()?;
    let system = flattened.linear_mna_from_devices()?;
    let solution = system.solve_exact()?;
    println!(
        "flattened conductance={}, OUT={}, replay={}",
        system.matrix[0][0], solution.candidate[0], solution.replay.accepted
    );
    for instance in &flattened.instances {
        println!(
            "{} resistance={} source={}",
            instance.id.as_str(),
            instance.parameters[0].value,
            instance.parameters[0].source
        );
    }
    Ok(())
}
