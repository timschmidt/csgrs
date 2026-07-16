use hypercircuit::{
    AdapterKind, BranchId, Circuit, CircuitAdapterReport, CircuitError, CircuitId, CircuitInstance,
    CircuitInstanceId, CircuitParameter, ComponentId, DeviceModel, DeviceModelId, DeviceModelKind,
    ElectromechanicalPort, ElectrothermalRcReport, ElectrothermalTraceFixture, EventPolicy,
    LinearMnaSystem, LinearStamp, Net, NetId, NonlinearDeviceKind, NonlinearDeviceReport, PartRef,
    PhysicalElectricalPort, PiecewiseLinearSegment, PinBinding, PinRef, Real, SwitchState,
    ThermalPort, TransientPolicy,
};
use proptest::prelude::*;

fn net(id: &str) -> NetId {
    NetId::new(id).unwrap()
}

fn branch(id: &str) -> BranchId {
    BranchId::new(id).unwrap()
}

fn component(id: &str) -> ComponentId {
    ComponentId::new(id).unwrap()
}

#[test]
fn voltage_source_and_resistor_stamp_replays_exact_candidate() {
    let vout = net("vout");
    let stamps = vec![
        LinearStamp::VoltageSource {
            component: component("v1"),
            branch: branch("iv1"),
            pos: Some(vout.clone()),
            neg: None,
            voltage: Real::from(5),
        },
        LinearStamp::Conductance {
            component: component("r1"),
            part: Some(PartRef::new("hyperparts:r0603").unwrap()),
            pos: Some(vout.clone()),
            neg: None,
            conductance: Real::from(2),
        },
    ];
    let system = LinearMnaSystem::from_stamps(vec![vout], &stamps).unwrap();

    let replay = system
        .replay_candidate(&[Real::from(5), Real::from(-10)])
        .unwrap();

    assert!(replay.accepted);
    assert!(replay.residuals.iter().all(|value| value == &Real::zero()));
}

#[test]
fn wrong_candidate_is_rejected_by_exact_residuals() {
    let node = net("n");
    let stamps = vec![LinearStamp::Conductance {
        component: component("g1"),
        part: None,
        pos: Some(node.clone()),
        neg: None,
        conductance: Real::from(1),
    }];
    let system = LinearMnaSystem::from_stamps(vec![node], &stamps).unwrap();

    let replay = system.replay_candidate(&[Real::from(1)]).unwrap();

    assert!(!replay.accepted);
    assert_eq!(replay.residuals, vec![Real::from(1)]);
}

#[test]
fn replay_validates_candidate_shape_and_adapter_status() {
    let system = LinearMnaSystem::from_stamps(vec![net("n")], &[]).unwrap();
    assert_eq!(
        system.replay_candidate(&[]).unwrap_err(),
        CircuitError::CandidateLengthMismatch
    );

    let report = CircuitAdapterReport {
        kind: AdapterKind::TransientDae,
        solver: "ida-fixture".into(),
        tolerance_policy: "proposal-only".into(),
        exact_replay_accepted: Some(false),
        notes: vec!["residual replay required".into()],
    };
    assert_eq!(report.exact_replay_accepted, Some(false));
}

#[test]
fn electrothermal_fixture_keeps_trace_and_material_handles_explicit() {
    let fixture = ElectrothermalTraceFixture {
        trace_handle: "hyperpath:trace/J1-1-to-R1-1".into(),
        material_handle: "hyperphysics:copper".into(),
        resistance: Real::from(2),
        thermal_coefficient: Real::from(4),
    };
    assert!(fixture.trace_handle.starts_with("hyperpath:"));
    assert!(fixture.material_handle.starts_with("hyperphysics:"));
}

#[test]
fn exact_circuit_carriers_lower_to_mna_without_float_policy() {
    let vout = net("vout");
    let ground = Net {
        id: net("0"),
        is_ground: true,
    };
    let resistor_model = DeviceModel {
        id: DeviceModelId::new("model:r").unwrap(),
        kind: DeviceModelKind::Resistor,
        parameters: vec![CircuitParameter {
            name: "conductance".into(),
            value: Real::from(2),
            unit: "S".into(),
            source: "test-fixture".into(),
        }],
    };
    let instance = CircuitInstance {
        id: CircuitInstanceId::new("r1").unwrap(),
        component: component("r1"),
        part: Some(PartRef::new("hyperparts:r0603").unwrap()),
        model: resistor_model.id.clone(),
        pins: vec![PinBinding {
            pin: PinRef::new("1").unwrap(),
            net: vout.clone(),
        }],
        parameters: resistor_model.parameters.clone(),
    };
    let circuit = Circuit::new(
        CircuitId::new("amp-fragment").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(ground)
    .with_net(Net {
        id: vout.clone(),
        is_ground: false,
    })
    .with_device_model(resistor_model)
    .with_instance(instance)
    .with_stamp(LinearStamp::Conductance {
        component: component("r1"),
        part: None,
        pos: Some(vout),
        neg: None,
        conductance: Real::from(2),
    });

    let system = circuit.linear_mna_system().unwrap();

    assert_eq!(system.unknowns.len(), 1);
    assert_eq!(system.matrix, vec![vec![Real::from(2)]]);
}

#[test]
fn vccs_stamp_uses_exact_control_voltage_columns() {
    let out = net("out");
    let ctrl = net("ctrl");
    let stamps = vec![LinearStamp::Vccs {
        component: component("g1"),
        pos: Some(out.clone()),
        neg: None,
        ctrl_pos: Some(ctrl.clone()),
        ctrl_neg: None,
        transconductance: Real::from(3),
    }];

    let system = LinearMnaSystem::from_stamps(vec![out, ctrl], &stamps).unwrap();

    assert_eq!(
        system.matrix,
        vec![
            vec![Real::zero(), Real::from(3)],
            vec![Real::zero(), Real::zero()]
        ]
    );
    assert_eq!(system.rhs, vec![Real::zero(), Real::zero()]);
}

#[test]
fn companion_stamp_combines_exact_conductance_and_history_source() {
    let node = net("n");
    let stamps = vec![LinearStamp::Companion {
        component: component("c1-companion"),
        pos: Some(node.clone()),
        neg: None,
        conductance: Real::from(4),
        history_current: Real::from(6),
    }];

    let system = LinearMnaSystem::from_stamps(vec![node], &stamps).unwrap();

    assert_eq!(system.matrix, vec![vec![Real::from(4)]]);
    assert_eq!(system.rhs, vec![Real::from(-6)]);
}

#[test]
fn duplicate_mna_unknowns_are_rejected_before_assembly() {
    let node = net("n");
    assert_eq!(
        LinearMnaSystem::from_stamps(vec![node.clone(), node], &[]).unwrap_err(),
        CircuitError::DuplicateUnknown
    );

    let stamps = vec![
        LinearStamp::VoltageSource {
            component: component("v1"),
            branch: branch("shared"),
            pos: None,
            neg: None,
            voltage: Real::from(1),
        },
        LinearStamp::VoltageSource {
            component: component("v2"),
            branch: branch("shared"),
            pos: None,
            neg: None,
            voltage: Real::from(2),
        },
    ];
    assert_eq!(
        LinearMnaSystem::from_stamps(Vec::new(), &stamps).unwrap_err(),
        CircuitError::DuplicateUnknown
    );
}

#[test]
fn coupling_ports_keep_circuit_and_physics_handles_explicit() {
    let electrical = PhysicalElectricalPort {
        handle: "trace-current".into(),
        net: net("vout"),
        physical_handle: "hyperpath:trace/r1".into(),
    };
    let thermal = ThermalPort {
        handle: "trace-temp".into(),
        physics_handle: "hyperphysics:thermal/r1".into(),
    };
    let actuator = ElectromechanicalPort {
        handle: "voice-coil".into(),
        physics_handle: "hyperphysics:body/coil".into(),
    };

    assert!(electrical.physical_handle.starts_with("hyperpath:"));
    assert!(thermal.physics_handle.starts_with("hyperphysics:"));
    assert!(actuator.physics_handle.starts_with("hyperphysics:"));
}

#[test]
fn electrothermal_rc_replay_computes_exact_resistance_and_heat() {
    let report = ElectrothermalRcReport::replay(
        component("r1"),
        Real::from(10),
        Real::from(2),
        Real::from(305),
        Real::from(300),
        Real::from(3),
        "hyperphysics:thermal/r1",
    )
    .unwrap();

    assert_eq!(report.adjusted_resistance, Real::from(110));
    assert_eq!(report.joule_heating, Real::from(990));
    assert_eq!(report.residual_block.residuals, vec![Real::zero()]);
    assert_eq!(report.residual_block.components, vec![component("r1")]);
}

#[test]
fn electrothermal_rc_rejects_negative_resistance_domains() {
    assert_eq!(
        ElectrothermalRcReport::replay(
            component("r1"),
            Real::from(-1),
            Real::zero(),
            Real::from(300),
            Real::from(300),
            Real::from(1),
            "hyperphysics:thermal/r1",
        )
        .unwrap_err(),
        CircuitError::NegativeResistance
    );
}

#[test]
fn nonlinear_reports_expose_domains_parameters_and_event_policy() {
    let diode = NonlinearDeviceReport::diode(
        component("d1"),
        vec![CircuitParameter {
            name: "is".into(),
            value: Real::from(1),
            unit: "A".into(),
            source: "test-model".into(),
        }],
    );
    assert_eq!(diode.kind, NonlinearDeviceKind::Diode);
    assert_eq!(diode.parameters.len(), 1);
    assert!(diode.domains[0].contains("Newton"));

    let switch = NonlinearDeviceReport::switch(
        component("sw1"),
        EventPolicy::ExactReplayRequired,
        SwitchState::Proposed,
    );
    assert_eq!(switch.event_policy, Some(EventPolicy::ExactReplayRequired));
    assert_eq!(switch.switch_state, Some(SwitchState::Proposed));
}

#[test]
fn piecewise_linear_report_keeps_exact_segments() {
    let report = NonlinearDeviceReport::piecewise_linear(
        component("pwl1"),
        vec![PiecewiseLinearSegment {
            lower: Real::from(0),
            upper: Real::from(10),
            slope: Real::from(2),
            intercept: Real::from(3),
        }],
    );

    assert_eq!(report.kind, NonlinearDeviceKind::PiecewiseLinear);
    assert_eq!(report.segments[0].slope, Real::from(2));
    assert!(report.slope_facts[0].contains("exact slope"));
}

proptest! {
    #[test]
    fn empty_component_ids_are_rejected(id in "\\PC*") {
        if id.is_empty() {
            prop_assert!(ComponentId::new(id).is_err());
        } else {
            prop_assert!(ComponentId::new(id).is_ok());
        }
    }

    #[test]
    fn generated_electrothermal_heat_is_i_squared_r(current in -100_i32..100, resistance in 0_i32..1000) {
        let report = ElectrothermalRcReport::replay(
            component("r-gen"),
            Real::from(resistance),
            Real::zero(),
            Real::from(300),
            Real::from(300),
            Real::from(current),
            "hyperphysics:thermal/r-gen",
        ).unwrap();

        prop_assert_eq!(report.adjusted_resistance, Real::from(resistance));
        prop_assert_eq!(report.joule_heating, Real::from(current * current * resistance));
    }

    #[test]
    fn generated_mna_stamps_match_kcl_reference(
        conductance in -20_i32..20,
        current in -20_i32..20,
        voltage in -20_i32..20,
        transconductance in -20_i32..20,
        companion_conductance in -20_i32..20,
        history_current in -20_i32..20,
    ) {
        let a = net("a");
        let b = net("b");
        let c = net("c");
        let stamps = vec![
            LinearStamp::Conductance {
                component: component("g"),
                part: None,
                pos: Some(a.clone()),
                neg: Some(b.clone()),
                conductance: Real::from(conductance),
            },
            LinearStamp::CurrentSource {
                component: component("i"),
                pos: Some(b.clone()),
                neg: Some(c.clone()),
                current: Real::from(current),
            },
            LinearStamp::VoltageSource {
                component: component("v"),
                branch: branch("iv"),
                pos: Some(a.clone()),
                neg: None,
                voltage: Real::from(voltage),
            },
            LinearStamp::Vccs {
                component: component("gm"),
                pos: Some(c.clone()),
                neg: Some(b.clone()),
                ctrl_pos: Some(a.clone()),
                ctrl_neg: None,
                transconductance: Real::from(transconductance),
            },
            LinearStamp::Companion {
                component: component("companion"),
                pos: Some(c.clone()),
                neg: None,
                conductance: Real::from(companion_conductance),
                history_current: Real::from(history_current),
            },
        ];

        let system = LinearMnaSystem::from_stamps(vec![a, b, c], &stamps).unwrap();
        let g = Real::from(conductance);
        let gm = Real::from(transconductance);
        let gc = Real::from(companion_conductance);
        let zero = Real::zero();
        let one = Real::one();
        let expected_matrix = vec![
            vec![g.clone(), -g.clone(), zero.clone(), one.clone()],
            vec![-g.clone() - &gm, g.clone(), zero.clone(), zero.clone()],
            vec![gm, zero.clone(), gc, zero.clone()],
            vec![one, zero.clone(), zero.clone(), zero],
        ];
        let expected_rhs = vec![
            Real::zero(),
            Real::from(-current),
            Real::from(current - history_current),
            Real::from(voltage),
        ];

        prop_assert_eq!(system.matrix, expected_matrix);
        prop_assert_eq!(system.rhs, expected_rhs);
    }
}
