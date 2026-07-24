use std::collections::BTreeMap;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitParameter,
    ComponentId, DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, DiodeNewtonPolicy,
    MnaUnknown, Net, NetId, PinBinding, PinElectricalKind, PinRef, Real, SourceStimulus,
    SourceWaveform, TransientAdaptation, TransientPolicy, TransientRunPolicy,
};

fn parameter(name: &str, value: Real, unit: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value,
        unit: unit.into(),
        source: "diode-transient-example".into(),
    }
}

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

fn instance(id: &str, model: DeviceModelId, pos: NetId, neg: NetId) -> CircuitInstance {
    CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: None,
        model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("+").unwrap(),
                net: pos,
            },
            PinBinding {
                pin: PinRef::new("-").unwrap(),
                net: neg,
            },
        ],
        parameters: Vec::new(),
    }
}

fn circuit() -> Circuit {
    let ground = NetId::new("GND").unwrap();
    let supply = NetId::new("SUPPLY").unwrap();
    let output = NetId::new("OUT").unwrap();
    let voltage = DeviceModelId::new("voltage").unwrap();
    let resistor = DeviceModelId::new("resistor").unwrap();
    let capacitor = DeviceModelId::new("capacitor").unwrap();
    let diode = DeviceModelId::new("diode").unwrap();
    Circuit::new(
        CircuitId::new("diode-transient").unwrap(),
        TransientPolicy::Trapezoidal,
        AdapterKind::TransientDae,
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
        id: output.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: voltage.clone(),
        kind: DeviceModelKind::VoltageSource,
        pins: pins(),
        parameters: Vec::new(),
    })
    .with_device_model(DeviceModel {
        id: resistor.clone(),
        kind: DeviceModelKind::Resistor,
        pins: pins(),
        parameters: vec![parameter("resistance", Real::one(), "ohm")],
    })
    .with_device_model(DeviceModel {
        id: capacitor.clone(),
        kind: DeviceModelKind::Capacitor,
        pins: pins(),
        parameters: vec![parameter("capacitance", Real::one(), "farad")],
    })
    .with_device_model(DeviceModel {
        id: diode.clone(),
        kind: DeviceModelKind::Diode,
        pins: pins(),
        parameters: vec![
            parameter(
                "saturation_current",
                (Real::one() / Real::from(10)).unwrap(),
                "ampere",
            ),
            parameter(
                "thermal_voltage",
                (Real::one() / Real::from(2)).unwrap(),
                "volt",
            ),
        ],
    })
    .with_instance(instance("V1", voltage, supply.clone(), ground.clone()))
    .with_instance(instance("R1", resistor, supply, output.clone()))
    .with_instance(instance("C1", capacitor, output.clone(), ground.clone()))
    .with_instance(instance("D1", diode, output, ground))
    .with_source_stimulus(SourceStimulus {
        component: ComponentId::new("V1").unwrap(),
        waveform: SourceWaveform::Step {
            initial: Real::zero(),
            final_value: Real::one(),
            at: Real::one(),
        },
    })
}

fn main() {
    let half = (Real::one() / Real::from(2)).unwrap();
    let tolerance = (Real::one() / Real::from(100_000_000)).unwrap();
    let report = circuit()
        .diode_transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::from(2),
                initial_timestep: half.clone(),
                minimum_timestep: half.clone(),
                maximum_timestep: half,
                maximum_accepted_steps: 4,
                maximum_rejected_steps: 1,
                adaptation: TransientAdaptation::Fixed,
            },
            &DiodeNewtonPolicy {
                maximum_iterations: 32,
                voltage_tolerance: tolerance.clone(),
                current_tolerance: tolerance,
                damping: Real::one(),
            },
            Default::default(),
            BTreeMap::new(),
        )
        .unwrap();
    let output = MnaUnknown::NetVoltage(NetId::new("OUT").unwrap());
    for ((time, value), evidence) in report
        .unknown_waveform(&output)
        .unwrap()
        .into_iter()
        .zip(&report.nonlinear_steps)
    {
        let output = value.to_f64_lossy().unwrap();
        let residual = evidence.maximum_kcl_residual.to_f64_lossy().unwrap();
        println!(
            "t={time}, OUT≈{output:.9}, newton={}, max-kcl≈{residual:.3e}, replay={}, exact-zero={}",
            evidence.iterations, evidence.replay_accepted, evidence.exact_zero
        );
    }
}
