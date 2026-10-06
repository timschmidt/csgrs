use std::collections::BTreeMap;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitParameter,
    ComponentId, DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, MosfetNewtonPolicy,
    MosfetPolarity, Net, NetId, PinBinding, PinElectricalKind, PinRef, Real, TransientPolicy,
};

fn pin(name: &str) -> DevicePin {
    DevicePin {
        pin: PinRef::new(name).unwrap(),
        kind: PinElectricalKind::Passive,
        optional: false,
    }
}

fn parameter(name: &str, value: Real, unit: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value,
        unit: unit.into(),
        source: "mosfet-dc-example".into(),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ground = NetId::new("GND").unwrap();
    let supply = NetId::new("VDD").unwrap();
    let gate = NetId::new("GATE").unwrap();
    let drain = NetId::new("DRAIN").unwrap();
    let voltage_model = DeviceModelId::new("voltage").unwrap();
    let resistor_model = DeviceModelId::new("resistor").unwrap();
    let mosfet_model = DeviceModelId::new("nmos").unwrap();
    let voltage_instance =
        |id: &str, net: NetId, value: i64, model: &DeviceModelId| CircuitInstance {
            id: CircuitInstanceId::new(id).unwrap(),
            component: ComponentId::new(id).unwrap(),
            part: None,
            model: model.clone(),
            pins: vec![
                PinBinding {
                    pin: PinRef::new("+").unwrap(),
                    net,
                },
                PinBinding {
                    pin: PinRef::new("-").unwrap(),
                    net: ground.clone(),
                },
            ],
            parameters: vec![parameter("voltage", Real::from(value), "volt")],
        };

    let circuit = Circuit::new(
        CircuitId::new("common-source").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_net(Net {
        id: supply.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: gate.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: drain.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: voltage_model.clone(),
        kind: DeviceModelKind::VoltageSource,
        pins: vec![pin("+"), pin("-")],
        parameters: Vec::new(),
    })
    .with_device_model(DeviceModel {
        id: resistor_model.clone(),
        kind: DeviceModelKind::Resistor,
        pins: vec![pin("1"), pin("2")],
        parameters: vec![parameter("resistance", Real::one(), "ohm")],
    })
    .with_device_model(DeviceModel {
        id: mosfet_model.clone(),
        kind: DeviceModelKind::Mosfet {
            polarity: MosfetPolarity::NChannel,
            drain: PinRef::new("D").unwrap(),
            gate: PinRef::new("G").unwrap(),
            source: PinRef::new("S").unwrap(),
        },
        pins: vec![pin("D"), pin("G"), pin("S")],
        parameters: vec![
            parameter("threshold_voltage", Real::one(), "volt"),
            parameter("transconductance_parameter", Real::one(), "ampere/volt^2"),
            parameter("channel_length_modulation", Real::zero(), "1/volt"),
        ],
    })
    .with_instance(voltage_instance("VDD1", supply.clone(), 5, &voltage_model))
    .with_instance(voltage_instance("VG1", gate.clone(), 2, &voltage_model))
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("R1").unwrap(),
        component: ComponentId::new("R1").unwrap(),
        part: None,
        model: resistor_model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("1").unwrap(),
                net: supply,
            },
            PinBinding {
                pin: PinRef::new("2").unwrap(),
                net: drain.clone(),
            },
        ],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("M1").unwrap(),
        component: ComponentId::new("M1").unwrap(),
        part: None,
        model: mosfet_model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("D").unwrap(),
                net: drain.clone(),
            },
            PinBinding {
                pin: PinRef::new("G").unwrap(),
                net: gate,
            },
            PinBinding {
                pin: PinRef::new("S").unwrap(),
                net: ground,
            },
        ],
        parameters: Vec::new(),
    });

    let report = circuit.solve_mosfet_dc(&MosfetNewtonPolicy::default(), &BTreeMap::new())?;
    let operating_point = &report.replay.operating_points[0];
    println!(
        "DRAIN={} region={:?} Id={} exact_replay={}",
        report.net_voltage(&drain).unwrap(),
        operating_point.region,
        operating_point.drain_current,
        report.replay.exact_zero
    );
    Ok(())
}
