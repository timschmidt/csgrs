use std::collections::BTreeMap;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitParameter,
    ComponentId, DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, DiodeNewtonPolicy,
    DiodeNewtonStatus, LinearStamp, Net, NetId, PiecewiseLinearDevice, PiecewiseLinearSegment,
    PiecewiseLinearSolveError, PinBinding, PinElectricalKind, PinRef, Real, TransientPolicy,
    solve_piecewise_linear,
};

fn fixture(current: i64) -> (NetId, Vec<LinearStamp>, PiecewiseLinearDevice) {
    let output = NetId::new("OUT").unwrap();
    let source = LinearStamp::CurrentSource {
        component: ComponentId::new("I1").unwrap(),
        pos: None,
        neg: Some(output.clone()),
        current: Real::from(current),
    };
    let device = PiecewiseLinearDevice {
        component: ComponentId::new("PWL1").unwrap(),
        pos: Some(output.clone()),
        neg: None,
        segments: vec![
            PiecewiseLinearSegment {
                lower: Real::zero(),
                upper: Real::one(),
                slope: Real::one(),
                intercept: Real::zero(),
            },
            PiecewiseLinearSegment {
                lower: Real::one(),
                upper: Real::from(10),
                slope: Real::from(2),
                intercept: Real::from(-1),
            },
        ],
    };
    (output, vec![source], device)
}

#[test]
fn exact_piecewise_linear_solver_enumerates_to_the_consistent_region() {
    let (output, stamps, device) = fixture(3);
    let report = solve_piecewise_linear(vec![output], &stamps, &[device], 2).unwrap();
    assert_eq!(report.active_segments, vec![1]);
    assert_eq!(report.candidates_tried, 2);
    assert_eq!(report.solution.candidate[0], Real::from(2));
    assert!(report.solution.replay.accepted);
}

#[test]
fn exact_piecewise_linear_solver_reports_shared_boundary_regions() {
    let (output, stamps, device) = fixture(1);
    let report = solve_piecewise_linear(vec![output], &stamps, &[device], 2).unwrap();
    assert_eq!(report.solution.candidate[0], Real::one());
    assert_eq!(report.alternative_regions, 1);
}

#[test]
fn exact_piecewise_linear_solver_enforces_enumeration_limit() {
    let (output, stamps, device) = fixture(3);
    assert!(matches!(
        solve_piecewise_linear(vec![output], &stamps, &[device], 1),
        Err(PiecewiseLinearSolveError::CombinationLimit {
            required: 2,
            maximum: 1
        })
    ));
}

fn two_terminal_pins(first: &str, second: &str) -> Vec<DevicePin> {
    [first, second]
        .into_iter()
        .map(|name| DevicePin {
            pin: PinRef::new(name).unwrap(),
            kind: PinElectricalKind::Passive,
            optional: false,
        })
        .collect()
}

fn parameter(name: &str, value: Real, unit: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value,
        unit: unit.into(),
        source: "test".into(),
    }
}

fn shockley_dc_fixture() -> Circuit {
    let ground = NetId::new("GND").unwrap();
    let supply = NetId::new("SUPPLY").unwrap();
    let output = NetId::new("OUT").unwrap();
    let voltage_model = DeviceModelId::new("voltage").unwrap();
    let resistor_model = DeviceModelId::new("resistor").unwrap();
    let diode_model = DeviceModelId::new("diode").unwrap();
    Circuit::new(
        CircuitId::new("shockley-dc").unwrap(),
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
        id: output.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: voltage_model.clone(),
        kind: DeviceModelKind::VoltageSource,
        pins: two_terminal_pins("+", "-"),
        parameters: vec![parameter("voltage", Real::one(), "volt")],
    })
    .with_device_model(DeviceModel {
        id: resistor_model.clone(),
        kind: DeviceModelKind::Resistor,
        pins: two_terminal_pins("1", "2"),
        parameters: vec![parameter("resistance", Real::one(), "ohm")],
    })
    .with_device_model(DeviceModel {
        id: diode_model.clone(),
        kind: DeviceModelKind::Diode,
        pins: two_terminal_pins("A", "K"),
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
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("V1").unwrap(),
        component: ComponentId::new("V1").unwrap(),
        part: None,
        model: voltage_model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("+").unwrap(),
                net: supply.clone(),
            },
            PinBinding {
                pin: PinRef::new("-").unwrap(),
                net: ground.clone(),
            },
        ],
        parameters: Vec::new(),
    })
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
                net: output.clone(),
            },
        ],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("D1").unwrap(),
        component: ComponentId::new("D1").unwrap(),
        part: None,
        model: diode_model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("A").unwrap(),
                net: output,
            },
            PinBinding {
                pin: PinRef::new("K").unwrap(),
                net: ground,
            },
        ],
        parameters: Vec::new(),
    })
}

#[test]
fn shockley_newton_uses_lossy_proposals_but_exact_true_law_replay() {
    let circuit = shockley_dc_fixture();
    let tolerance = (Real::one() / Real::from(100_000_000)).unwrap();
    let report = circuit
        .solve_diode_dc(
            &DiodeNewtonPolicy {
                maximum_iterations: 32,
                voltage_tolerance: tolerance.clone(),
                current_tolerance: tolerance,
                damping: Real::one(),
            },
            &BTreeMap::new(),
        )
        .unwrap();
    assert_eq!(report.status, DiodeNewtonStatus::Converged);
    assert!(report.used_lossy_linearization);
    assert!(report.replay.accepted);
    assert!(!report.iterations.is_empty());
    assert!(
        report
            .iterations
            .iter()
            .all(|iteration| iteration.linear_proposal_replay_accepted
                && iteration.linearizations.len() == 1)
    );
    let output = report.net_voltage(&NetId::new("OUT").unwrap()).unwrap();
    assert!(output > &Real::zero());
    assert!(output < &Real::one());
    assert!(output.to_f64_lossy().is_some());
}

#[test]
fn shockley_newton_returns_an_audited_iteration_limit() {
    let tiny = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    let report = shockley_dc_fixture()
        .solve_diode_dc(
            &DiodeNewtonPolicy {
                maximum_iterations: 1,
                voltage_tolerance: tiny.clone(),
                current_tolerance: tiny,
                damping: Real::one(),
            },
            &BTreeMap::new(),
        )
        .unwrap();
    assert_eq!(report.status, DiodeNewtonStatus::IterationLimit);
    assert_eq!(report.iterations.len(), 1);
    assert!(report.iterations[0].linear_proposal_replay_accepted);
}

#[cfg(feature = "interchange")]
#[test]
fn retained_diode_model_round_trips_through_semantic_json() {
    let document = hypercircuit::SemanticDocument::new(shockley_dc_fixture(), None).unwrap();
    let json = document.to_json_pretty().unwrap();
    assert!(json.contains("\"Diode\""));
    assert_eq!(
        hypercircuit::SemanticDocument::from_json(&json).unwrap(),
        document
    );
}
