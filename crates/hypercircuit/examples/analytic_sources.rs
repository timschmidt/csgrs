use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitParameter,
    ComponentId, DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, MnaUnknown, Net, NetId,
    PinBinding, PinElectricalKind, PinRef, Real, SourceStimulus, SourceWaveform,
    TransientAdaptation, TransientPolicy, TransientRunPolicy,
};

fn pins() -> Vec<DevicePin> {
    ["+", "-"]
        .into_iter()
        .map(|name| DevicePin {
            pin: PinRef::new(name).unwrap(),
            kind: PinElectricalKind::Passive,
            optional: false,
        })
        .collect()
}

fn instance(id: &str, model: DeviceModelId, output: &NetId, ground: &NetId) -> CircuitInstance {
    CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: None,
        model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("+").unwrap(),
                net: output.clone(),
            },
            PinBinding {
                pin: PinRef::new("-").unwrap(),
                net: ground.clone(),
            },
        ],
        parameters: Vec::new(),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ground = NetId::new("GND").unwrap();
    let output = NetId::new("OUT").unwrap();
    let source_model = DeviceModelId::new("voltage-source").unwrap();
    let resistor_model = DeviceModelId::new("load").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("analytic-source").unwrap(),
        TransientPolicy::GearBdf { order: 1 },
        AdapterKind::TransientDae,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: source_model.clone(),
        kind: DeviceModelKind::VoltageSource,
        pins: pins(),
        parameters: Vec::new(),
    })
    .with_device_model(DeviceModel {
        id: resistor_model.clone(),
        kind: DeviceModelKind::Resistor,
        pins: pins(),
        parameters: vec![CircuitParameter {
            name: "resistance".into(),
            value: Real::one(),
            unit: "ohm".into(),
            source: "analytic-source-example".into(),
        }],
    })
    .with_instance(instance("V1", source_model, &output, &ground))
    .with_instance(instance("R1", resistor_model, &output, &ground))
    .with_source_stimulus(SourceStimulus {
        component: ComponentId::new("V1").unwrap(),
        waveform: SourceWaveform::Sine {
            offset: Real::zero(),
            amplitude: Real::one(),
            frequency: Real::one(),
            delay: (Real::one() / Real::from(2))?,
            damping: Real::zero(),
            phase_degrees: Real::zero(),
        },
    });

    let report = circuit.transient_run(
        &TransientRunPolicy {
            start_time: Real::zero(),
            stop_time: Real::from(2),
            initial_timestep: (Real::one() / Real::from(4))?,
            minimum_timestep: (Real::one() / Real::from(4))?,
            maximum_timestep: (Real::one() / Real::from(4))?,
            maximum_accepted_steps: 8,
            maximum_rejected_steps: 1,
            adaptation: TransientAdaptation::Fixed,
        },
        Default::default(),
    )?;

    for (time, voltage) in report
        .unknown_waveform(&MnaUnknown::NetVoltage(output))
        .expect("OUT is a retained MNA unknown")
    {
        println!("t={time} OUT={voltage}");
    }
    Ok(())
}
