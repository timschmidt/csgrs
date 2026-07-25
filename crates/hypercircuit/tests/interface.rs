use hypercircuit::{
    AdapterKind, BundleEndpointId, BundleMemberId, BundlePortBinding, Circuit, CircuitEventCause,
    CircuitEventKind, CircuitEventPhase, CircuitEventRequest, CircuitEventTarget, CircuitId,
    CircuitLibrary, CircuitPort, LogicValue, Modport, ModportId, ModportMember, Net, NetId,
    PortDirection, PortId, PortSignalType, Real, SignalBundle, SignalBundleBindingError,
    SignalBundleEndpoint, SignalBundleId, SignalBundleLibrary, SignalBundleMember,
    SignalBundleValidationIssue, SubcircuitInstance, SubcircuitInstanceId, TransientPolicy,
};

fn circuit(id: &str, vcc_direction: PortDirection) -> Circuit {
    let vcc = NetId::new("vcc").unwrap();
    let gnd = NetId::new("gnd").unwrap();
    Circuit::new(
        CircuitId::new(id).unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: vcc.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: gnd.clone(),
        is_ground: true,
    })
    .with_port(CircuitPort {
        id: PortId::new("vcc").unwrap(),
        net: vcc,
        direction: vcc_direction,
        optional: false,
    })
    .with_port(CircuitPort {
        id: PortId::new("gnd").unwrap(),
        net: gnd,
        direction: PortDirection::Ground,
        optional: false,
    })
}

#[test]
fn typed_bundle_members_must_match_their_bound_circuit_ports() {
    let board = circuit("typed-board", PortDirection::Input);
    let circuits = CircuitLibrary {
        root: board.id.clone(),
        circuits: vec![board],
    };
    let typed = SignalBundle::new_typed(
        SignalBundleId::new("Power").unwrap(),
        vec![
            SignalBundleMember::new(BundleMemberId::new("vcc").unwrap(), PortSignalType::Logic),
            SignalBundleMember::new(BundleMemberId::new("gnd").unwrap(), PortSignalType::Real),
        ],
    )
    .with_modport(Modport::new(
        ModportId::new("sink").unwrap(),
        vec![
            ModportMember {
                member: BundleMemberId::new("vcc").unwrap(),
                direction: PortDirection::Input,
            },
            ModportMember {
                member: BundleMemberId::new("gnd").unwrap(),
                direction: PortDirection::Ground,
            },
        ],
    ));
    let report = SignalBundleLibrary::new()
        .with_bundle(typed)
        .with_endpoint(endpoint("typed-board", "power", "sink"))
        .validate(&circuits);

    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        SignalBundleValidationIssue::EndpointPortSignalTypeMismatch {
            member,
            expected: PortSignalType::Logic,
            actual: PortSignalType::Real,
            ..
        } if member.as_str() == "vcc"
    )));

    let invalid_width = SignalBundleLibrary::new().with_bundle(SignalBundle::new_typed(
        SignalBundleId::new("Data").unwrap(),
        vec![SignalBundleMember::new(
            BundleMemberId::new("payload").unwrap(),
            PortSignalType::Bus { width: Some(0) },
        )],
    ));
    let report = invalid_width.validate(&circuits);
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        SignalBundleValidationIssue::InvalidBundleMemberSignalType { member, .. }
            if member.as_str() == "payload"
    )));
}

fn power_bundle() -> SignalBundle {
    let source = Modport::new(
        ModportId::new("source").unwrap(),
        vec![
            ModportMember {
                member: BundleMemberId::new("vcc").unwrap(),
                direction: PortDirection::Output,
            },
            ModportMember {
                member: BundleMemberId::new("gnd").unwrap(),
                direction: PortDirection::Ground,
            },
        ],
    );
    let sink = source.dual(ModportId::new("sink").unwrap());
    SignalBundle::new(
        SignalBundleId::new("Power").unwrap(),
        vec![
            BundleMemberId::new("vcc").unwrap(),
            BundleMemberId::new("gnd").unwrap(),
        ],
    )
    .with_modport(source)
    .with_modport(sink)
}

fn endpoint(circuit: &str, endpoint: &str, modport: &str) -> SignalBundleEndpoint {
    SignalBundleEndpoint::new(
        BundleEndpointId::new(endpoint).unwrap(),
        CircuitId::new(circuit).unwrap(),
        SignalBundleId::new("Power").unwrap(),
        ModportId::new(modport).unwrap(),
        vec![
            BundlePortBinding {
                member: BundleMemberId::new("vcc").unwrap(),
                port: PortId::new("vcc").unwrap(),
            },
            BundlePortBinding {
                member: BundleMemberId::new("gnd").unwrap(),
                port: PortId::new("gnd").unwrap(),
            },
        ],
    )
}

#[test]
fn modport_dual_flips_directional_members_and_retains_symmetric_roles() {
    let bundle = power_bundle();
    let source = bundle.modport(&ModportId::new("source").unwrap()).unwrap();
    let sink = bundle.modport(&ModportId::new("sink").unwrap()).unwrap();

    assert_eq!(
        source.direction(&BundleMemberId::new("vcc").unwrap()),
        Some(PortDirection::Output)
    );
    assert_eq!(
        sink.direction(&BundleMemberId::new("vcc").unwrap()),
        Some(PortDirection::Input)
    );
    assert_eq!(
        sink.direction(&BundleMemberId::new("gnd").unwrap()),
        Some(PortDirection::Ground)
    );
    assert!(PortDirection::Output.is_dual_to(PortDirection::Input));
    assert!(PortDirection::Passive.is_dual_to(PortDirection::Passive));
}

#[test]
fn dual_bundle_endpoints_lower_to_existing_hierarchy_port_bindings() {
    let child = circuit("regulator", PortDirection::Output);
    let child_id = child.id.clone();
    let root = circuit("board", PortDirection::Input).with_subcircuit(SubcircuitInstance {
        id: SubcircuitInstanceId::new("u1").unwrap(),
        circuit: child_id,
        ports: Vec::new(),
        parameter_overrides: Vec::new(),
    });
    let mut circuits = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, child],
    };
    let bundles = SignalBundleLibrary::new()
        .with_bundle(power_bundle())
        .with_endpoint(endpoint("board", "power-in", "sink"))
        .with_endpoint(endpoint("regulator", "power-out", "source"));

    assert!(bundles.validate(&circuits).is_valid());
    let lowered = bundles
        .bind_subcircuit(
            &mut circuits,
            &CircuitId::new("board").unwrap(),
            &SubcircuitInstanceId::new("u1").unwrap(),
            &BundleEndpointId::new("power-in").unwrap(),
            &BundleEndpointId::new("power-out").unwrap(),
        )
        .unwrap();

    assert_eq!(lowered.len(), 2);
    assert_eq!(lowered[0].port.as_str(), "vcc");
    assert_eq!(lowered[0].net.as_str(), "vcc");
    assert_eq!(lowered[1].port.as_str(), "gnd");
    assert_eq!(lowered[1].net.as_str(), "gnd");
    assert!(circuits.validate().is_valid());
    assert!(circuits.flatten().is_ok());

    // Lowering is idempotent and does not duplicate ordinary port bindings.
    bundles
        .bind_subcircuit(
            &mut circuits,
            &CircuitId::new("board").unwrap(),
            &SubcircuitInstanceId::new("u1").unwrap(),
            &BundleEndpointId::new("power-in").unwrap(),
            &BundleEndpointId::new("power-out").unwrap(),
        )
        .unwrap();
    assert_eq!(circuits.circuits[0].subcircuits[0].ports.len(), 2);
}

#[test]
fn validation_reports_incomplete_views_and_endpoint_direction_mismatches() {
    let malformed = SignalBundle::new(
        SignalBundleId::new("Power").unwrap(),
        vec![
            BundleMemberId::new("vcc").unwrap(),
            BundleMemberId::new("gnd").unwrap(),
        ],
    )
    .with_modport(Modport::new(
        ModportId::new("source").unwrap(),
        vec![ModportMember {
            member: BundleMemberId::new("vcc").unwrap(),
            direction: PortDirection::Output,
        }],
    ));
    let board = circuit("board", PortDirection::Input);
    let circuits = CircuitLibrary {
        root: board.id.clone(),
        circuits: vec![board],
    };
    let bundles = SignalBundleLibrary::new()
        .with_bundle(malformed)
        .with_endpoint(endpoint("board", "power", "source"));
    let report = bundles.validate(&circuits);

    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        SignalBundleValidationIssue::MissingModportMember { member, .. }
            if member.as_str() == "gnd"
    )));
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        SignalBundleValidationIssue::EndpointPortDirectionMismatch { member, .. }
            if member.as_str() == "vcc"
    )));
}

#[test]
fn binding_rejects_two_endpoints_presenting_the_same_direction() {
    let child = circuit("consumer", PortDirection::Input);
    let root = circuit("board", PortDirection::Input).with_subcircuit(SubcircuitInstance {
        id: SubcircuitInstanceId::new("u1").unwrap(),
        circuit: child.id.clone(),
        ports: Vec::new(),
        parameter_overrides: Vec::new(),
    });
    let mut circuits = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, child],
    };
    let bundles = SignalBundleLibrary::new()
        .with_bundle(power_bundle())
        .with_endpoint(endpoint("board", "board-power", "sink"))
        .with_endpoint(endpoint("consumer", "consumer-power", "sink"));

    let error = bundles
        .bind_subcircuit(
            &mut circuits,
            &CircuitId::new("board").unwrap(),
            &SubcircuitInstanceId::new("u1").unwrap(),
            &BundleEndpointId::new("board-power").unwrap(),
            &BundleEndpointId::new("consumer-power").unwrap(),
        )
        .unwrap_err();
    assert!(matches!(
        error,
        SignalBundleBindingError::NonDualMember { member, .. }
            if member.as_str() == "vcc"
    ));
}

#[test]
fn event_addresses_validate_against_bundle_members_and_hierarchy_paths() {
    let child = circuit("regulator", PortDirection::Output);
    let root = circuit("board", PortDirection::Input).with_subcircuit(SubcircuitInstance {
        id: SubcircuitInstanceId::new("u1").unwrap(),
        circuit: child.id.clone(),
        ports: Vec::new(),
        parameter_overrides: Vec::new(),
    });
    let library = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root.clone(), child],
    };
    let bundles = SignalBundleLibrary::new()
        .with_bundle(power_bundle())
        .with_endpoint(endpoint("board", "power-in", "sink"));
    let request = CircuitEventRequest {
        time: Real::zero(),
        phase: CircuitEventPhase::Detection,
        source: Some(CircuitEventTarget::Hierarchy(vec![
            SubcircuitInstanceId::new("u1").unwrap(),
        ])),
        target: CircuitEventTarget::BundleMember {
            circuit: root.id.clone(),
            endpoint: BundleEndpointId::new("power-in").unwrap(),
            member: BundleMemberId::new("vcc").unwrap(),
        },
        kind: CircuitEventKind::DigitalTransition {
            value: LogicValue::High,
        },
        cause: CircuitEventCause::Authored {
            provenance: "bundle-address-test".into(),
        },
    };

    assert!(request.validate_against(&root, Some(&bundles)).is_empty());
    assert!(request.validate_hierarchy(&library).is_empty());
}
