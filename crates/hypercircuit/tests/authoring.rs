#![cfg(feature = "layout")]

use std::collections::BTreeMap;

use hypercircuit::{
    AuthoringAction, AuthoringTarget, BoardOutline, BoardSide, BusSliceOrder,
    CircuitValidationIssue, Design, DesignBuildError, DesignValidationIssue, DifferentialPairRule,
    DiodeNewtonPolicy, DiodeNewtonStatus, Footprint, Keepout, KeepoutScope, KiCadExportOptions,
    KiCadSchematicExportOptions, KiCadSchematicImportReport, LengthTuningRule, MnaUnknown,
    MosfetNewtonPolicy, MosfetNewtonStatus, NetClassRule, NetHandle, NetId, PartDefinition,
    PartInstance, PartSymbolUnit, PcbStackup, PhaseTuningGroupRule, PhaseTuningStatus,
    PlacementRule, PortDirection, RailKind, Real, Route, RoutingProblemReport, SchematicPinSide,
    SchematicPoint, SchematicSvgOptions, SourceWaveform, SourceWaveformPoint, Symbol, SymbolPin,
    SymbolUnitPlacement, TransientAdaptation, TransientPolicy, TransientRunPolicy,
    TransientRunStatus, Via, ViaMaskIntent, ViaStyleRule, Zone, parts, pin,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

#[test]
fn reusable_part_definition_lowers_library_records_once_for_many_instances() {
    let mut design = Design::new(
        "reusable-parts",
        BoardOutline::rectangle(Real::from(20), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let common = design.ground("GND").unwrap();
    let symbol_unit = PartSymbolUnit::new(1, Real::from(6), Real::from(3))
        .pin(SymbolPin::new(
            "1",
            SchematicPoint::new(Real::from(-4), Real::zero()),
            SchematicPinSide::Left,
        ))
        .pin(SymbolPin::new(
            "2",
            SchematicPoint::new(Real::from(4), Real::zero()),
            SchematicPinSide::Right,
        ))
        .rectangular_body(Real::one())
        .unwrap();
    let resistor = design
        .define_part(
            PartDefinition::new("resistor-0603", "0603 resistor")
                .model_kind(hypercircuit::DeviceModelKind::Resistor)
                .part_ref("hyperparts:resistor-0603")
                .pin(pin("1").pad("1"))
                .pin(pin("2").pad("2"))
                .symbol_name("R")
                .symbol_unit(symbol_unit)
                .footprint(Footprint::two_pad_smd(
                    Real::one(),
                    Real::one(),
                    Real::from(2),
                    vec![TraceLayer(0)],
                )),
        )
        .unwrap();
    let first = design
        .instantiate(
            &resistor,
            PartInstance::new("R1")
                .parameter("resistance", Real::from(1_000), "ohm")
                .symbol(SymbolUnitPlacement::new(
                    1,
                    SchematicPoint::new(Real::from(4), Real::from(3)),
                ))
                .at(point(5, 5)),
        )
        .unwrap();
    let second = design
        .instantiate(
            &resistor,
            PartInstance::new("R2")
                .parameter("resistance", Real::from(2_000), "ohm")
                .symbol(SymbolUnitPlacement::new(
                    1,
                    SchematicPoint::new(Real::from(14), Real::from(3)),
                ))
                .at(point(12, 5)),
        )
        .unwrap();
    design
        .connect(
            &common,
            [
                first.pin("1").unwrap(),
                first.pin("2").unwrap(),
                second.pin("1").unwrap(),
                second.pin("2").unwrap(),
            ],
        )
        .unwrap();

    assert_eq!(resistor.model_id().as_str(), "resistor-0603.model");
    assert_eq!(
        resistor
            .symbol_definition_id()
            .expect("symbol definition")
            .as_str(),
        "resistor-0603.symbol"
    );
    assert_eq!(
        resistor.land_pattern_id().expect("land pattern").as_str(),
        "resistor-0603.footprint"
    );
    let checked = design.finish().unwrap();
    assert_eq!(checked.circuit.device_models.len(), 1);
    assert_eq!(checked.circuit.instances.len(), 2);
    assert!(
        checked
            .circuit
            .instances
            .iter()
            .all(|instance| instance.model.as_str() == "resistor-0603.model")
    );
    assert_eq!(checked.schematic.symbol_definitions.len(), 1);
    assert_eq!(checked.schematic.symbols.len(), 2);
    assert_eq!(checked.layout.land_patterns.len(), 1);
    assert_eq!(checked.layout.placements.len(), 2);
    let kicad = checked
        .schematic
        .export_kicad_schematic(&checked.circuit, KiCadSchematicExportOptions::default())
        .unwrap();
    assert_eq!(kicad.symbols.len(), 2);
    assert_eq!(kicad.symbols[0].library_id, kicad.symbols[1].library_id);
    let imported =
        KiCadSchematicImportReport::from_str(&kicad.schematic, &checked.circuit).unwrap();
    assert_eq!(
        imported.layout.symbol_definitions,
        checked.schematic.symbol_definitions
    );
    #[cfg(feature = "interchange")]
    {
        let document = hypercircuit::SemanticDocument::new(
            checked.circuit.clone(),
            Some(checked.schematic.clone()),
        )
        .unwrap()
        .with_pcb(checked.layout.clone())
        .unwrap();
        let json = document.to_json_pretty().unwrap();
        assert_eq!(
            hypercircuit::SemanticDocument::from_json(&json).unwrap(),
            document
        );
    }
    assert_eq!(
        checked.circuit.lower_linear_devices().stamps.len(),
        2,
        "instance parameters override one shared executable model"
    );
}

#[test]
fn fluent_design_lowers_to_checked_simulation_and_layout_intent() {
    let mut design = Design::new(
        "fluent-divider",
        BoardOutline::rectangle(Real::from(20), Real::from(12)),
        PcbStackup::two_layer(
            (Real::from(35) / Real::from(1_000)).unwrap(),
            (Real::from(153) / Real::from(100)).unwrap(),
            Some("hyperphysics:copper".into()),
            Some("hyperphysics:FR4".into()),
        ),
    )
    .unwrap();
    let supply = design
        .rail(
            "VCC",
            Some(Real::from(5)),
            Some(Real::one()),
            RailKind::Power,
        )
        .unwrap();
    let ground = design.ground("GND").unwrap();
    let source = design
        .add(
            parts::voltage_source("V1", Real::from(5)).symbol(
                Symbol::new(
                    SchematicPoint::new(Real::from(2), Real::from(2)),
                    Real::from(4),
                    Real::from(3),
                )
                .pin(SymbolPin::new(
                    "pos",
                    SchematicPoint::new(Real::from(2), Real::zero()),
                    SchematicPinSide::Right,
                ))
                .pin(SymbolPin::new(
                    "neg",
                    SchematicPoint::new(Real::from(-2), Real::zero()),
                    SchematicPinSide::Left,
                )),
            ),
        )
        .unwrap();
    let resistor = design
        .add(
            parts::resistor("R1", Real::from(1_000))
                .symbol(Symbol::two_pin_horizontal(
                    SchematicPoint::new(Real::from(10), Real::from(2)),
                    Real::from(2),
                    Real::from(3),
                    Real::from(2),
                ))
                .footprint(Footprint::two_pad_smd(
                    Real::one(),
                    Real::one(),
                    Real::from(2),
                    vec![TraceLayer(0)],
                ))
                .at(point(10, 6)),
        )
        .unwrap();
    design
        .connect(
            &supply,
            [source.pin("pos").unwrap(), resistor.pin("1").unwrap()],
        )
        .unwrap();
    design
        .connect(
            &ground,
            [source.pin("neg").unwrap(), resistor.pin("2").unwrap()],
        )
        .unwrap();

    let checked = design.finish().unwrap();
    assert_eq!(checked.circuit.nets.len(), 2);
    assert_eq!(checked.circuit.instances.len(), 2);
    assert_eq!(checked.schematic.symbols.len(), 2);
    assert_eq!(checked.schematic.wires.len(), 2);
    assert!(checked.schematic.validate(&checked.circuit).is_valid());
    assert!(
        checked
            .schematic
            .to_svg(&checked.circuit, SchematicSvgOptions::default())
            .unwrap()
            .svg
            .contains("R1")
    );
    assert_eq!(checked.layout.land_patterns.len(), 1);
    assert_eq!(checked.layout.placements.len(), 1);
    assert_eq!(checked.layout.land_patterns[0].pin_map.len(), 2);
    assert!(checked.source_map.traces.iter().any(|trace| {
        trace.action == AuthoringAction::Connect
            && matches!(
                &trace.target,
                AuthoringTarget::Connection { instance, pin }
                    if instance.as_str() == "R1" && pin.as_str() == "1"
            )
    }));
    let lowering = checked.circuit.lower_linear_devices();
    assert!(lowering.issues.is_empty());
    assert_eq!(lowering.stamps.len(), 2);
    let routing = RoutingProblemReport::from_layout(&checked.circuit, &checked.layout).unwrap();
    assert!(routing.omissions.is_empty());
    let kicad = checked
        .layout
        .export_kicad(&checked.circuit, KiCadExportOptions::default())
        .unwrap();
    assert!(kicad.board.contains("(property \"Reference\" \"R1\")"));
}

#[test]
fn connection_batches_are_atomic_and_part_physical_requirements_fail_early() {
    let mut design = Design::new(
        "authoring-errors",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let signal = design.signal("SIGNAL").unwrap();
    let resistor = design.add(parts::resistor("R1", Real::from(100))).unwrap();
    let pin = resistor.pin("1").unwrap();
    assert!(matches!(
        design.connect(&signal, [pin.clone(), pin]),
        Err(DesignBuildError::DuplicateConnection { .. })
    ));
    assert!(design.circuit().instances[0].pins.is_empty());

    let missing_placement = parts::resistor("R2", Real::from(100)).footprint(
        Footprint::two_pad_smd(Real::one(), Real::one(), Real::one(), vec![TraceLayer(0)]),
    );
    assert!(matches!(
        design.add(missing_placement),
        Err(DesignBuildError::MissingPlacement(instance)) if instance == "R2"
    ));
    assert_eq!(design.circuit().instances.len(), 1);
    assert!(
        design
            .circuit()
            .device_models
            .iter()
            .all(|model| model.id.as_str() != "R2.model")
    );

    let incomplete_symbol = parts::resistor("R3", Real::from(100)).symbol(
        Symbol::new(
            SchematicPoint::new(Real::zero(), Real::zero()),
            Real::from(3),
            Real::from(2),
        )
        .pin(SymbolPin::new(
            "1",
            SchematicPoint::new(Real::from(-2), Real::zero()),
            SchematicPinSide::Left,
        )),
    );
    assert!(matches!(
        design.add(incomplete_symbol),
        Err(DesignBuildError::MissingSymbolPin { instance, pin })
            if instance == "R3" && pin == "2"
    ));
    assert!(
        design
            .schematic()
            .symbols
            .iter()
            .all(|symbol| symbol.instance.as_str() != "R3")
    );
}

#[test]
fn typed_handles_cannot_cross_design_boundaries() {
    let mut left = Design::new(
        "left",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let mut right = Design::new(
        "left",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let left_net = left.signal("SIGNAL").unwrap();
    let right_net = right.signal("SIGNAL").unwrap();
    let left_resistor = left.add(parts::resistor("R1", Real::from(100))).unwrap();
    let right_resistor = right.add(parts::resistor("R1", Real::from(100))).unwrap();

    assert_eq!(
        left.connect(&right_net, [left_resistor.pin("1").unwrap()]),
        Err(DesignBuildError::ForeignHandle)
    );
    assert_eq!(
        left.connect(&left_net, [right_resistor.pin("1").unwrap()]),
        Err(DesignBuildError::ForeignHandle)
    );
    assert!(left.circuit().instances[0].pins.is_empty());
}

#[test]
fn structural_diagnostics_point_back_to_part_and_escape_hatch_call_sites() {
    let mut design = Design::new(
        "source-aware",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let signal = design.signal("SIGNAL").unwrap();
    let declaration_line = line!() + 1;
    let resistor = design.add(parts::resistor("R1", Real::from(100))).unwrap();
    design
        .connect(&signal, [resistor.pin("1").unwrap()])
        .unwrap();

    let report = design.check();
    let missing_pin = report
        .diagnostics
        .iter()
        .find(|diagnostic| {
            matches!(
                &diagnostic.issue,
                DesignValidationIssue::Circuit(
                    CircuitValidationIssue::MissingRequiredInstancePin { instance, pin }
                ) if instance.as_str() == "R1" && pin.as_str() == "2"
            )
        })
        .expect("unconnected required pin must be diagnosed");
    assert!(missing_pin.sources.iter().any(|source| {
        source.file.ends_with("tests/authoring.rs") && source.line == declaration_line
    }));

    let mutation_line = line!() + 1;
    design.circuit_mut().instances[0].pins.clear();
    let report = design.check();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        matches!(
            &diagnostic.issue,
            DesignValidationIssue::Circuit(
                CircuitValidationIssue::MissingRequiredInstancePin { instance, .. }
            ) if instance.as_str() == "R1"
        ) && diagnostic.sources.iter().any(|source| {
            source.file.ends_with("tests/authoring.rs") && source.line == mutation_line
        })
    }));
}

#[cfg(feature = "interchange")]
#[test]
fn authoring_source_map_round_trips_independently_of_semantic_ir() {
    let design = Design::new(
        "source-map-json",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let json = serde_json::to_string(design.source_map()).unwrap();
    let decoded: hypercircuit::DesignSourceMap = serde_json::from_str(&json).unwrap();
    assert_eq!(&decoded, design.source_map());
}

#[test]
fn incremental_connections_chain_visible_symbols_and_report_schematic_mutations() {
    let mut design = Design::new(
        "incremental-schematic",
        BoardOutline::rectangle(Real::from(20), Real::from(12)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let signal = design.signal("SIGNAL").unwrap();
    let ground = design.ground("GND").unwrap();
    let mut resistors = Vec::new();
    for index in 0..3 {
        resistors.push(
            design
                .add(
                    parts::resistor(format!("R{}", index + 1), Real::from(100)).symbol(
                        Symbol::two_pin_horizontal(
                            SchematicPoint::new(Real::from(index * 6), Real::zero()),
                            Real::from(2),
                            Real::from(3),
                            Real::from(2),
                        ),
                    ),
                )
                .unwrap(),
        );
    }
    for resistor in &resistors {
        design
            .connect(&signal, [resistor.pin("1").unwrap()])
            .unwrap();
    }
    design
        .connect(
            &ground,
            resistors.iter().map(|resistor| resistor.pin("2").unwrap()),
        )
        .unwrap();
    assert_eq!(design.schematic().wires.len(), 4);
    assert!(design.check().is_valid());

    let mutation_line = line!() + 1;
    design.schematic_mut().symbol_definitions[0].units[0].body_width = Real::zero();
    let report = design.check();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        matches!(
            &diagnostic.issue,
            DesignValidationIssue::Schematic(
                hypercircuit::SchematicValidationIssue::InvalidSymbolUnitBody { .. }
            )
        ) && diagnostic.sources.iter().any(|source| {
            source.file.ends_with("tests/authoring.rs") && source.line == mutation_line
        })
    }));
}

#[test]
fn fluent_nonlinear_parts_extract_and_execute_retained_device_laws() {
    let mut diode_design = Design::new(
        "fluent-diode",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let supply = diode_design.signal("SUPPLY").unwrap();
    let output = diode_design.signal("OUT").unwrap();
    let ground = diode_design.ground("GND").unwrap();
    let voltage = diode_design
        .add(parts::voltage_source("V1", Real::one()))
        .unwrap();
    let resistor = diode_design
        .add(parts::resistor("R1", Real::one()))
        .unwrap();
    let diode = diode_design
        .add(parts::diode(
            "D1",
            (Real::one() / Real::from(10)).unwrap(),
            (Real::one() / Real::from(2)).unwrap(),
        ))
        .unwrap();
    diode_design
        .connect(
            &supply,
            [voltage.pin("pos").unwrap(), resistor.pin("1").unwrap()],
        )
        .unwrap();
    diode_design
        .connect(
            &output,
            [resistor.pin("2").unwrap(), diode.pin("A").unwrap()],
        )
        .unwrap();
    diode_design
        .connect(
            &ground,
            [voltage.pin("neg").unwrap(), diode.pin("K").unwrap()],
        )
        .unwrap();
    let diode_circuit = diode_design.finish().unwrap().circuit;
    assert_eq!(diode_circuit.shockley_diodes().unwrap().len(), 1);
    let diode_report = diode_circuit
        .solve_diode_dc(&DiodeNewtonPolicy::default(), &BTreeMap::new())
        .unwrap();
    assert_eq!(diode_report.status, DiodeNewtonStatus::Converged);
    assert!(diode_report.replay.accepted);

    let mut mosfet_design = Design::new(
        "fluent-mosfet",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let vdd = mosfet_design.signal("VDD").unwrap();
    let gate = mosfet_design.signal("GATE").unwrap();
    let drain = mosfet_design.signal("DRAIN").unwrap();
    let ground = mosfet_design.ground("GND").unwrap();
    let supply = mosfet_design
        .add(parts::voltage_source("VDD1", Real::from(5)))
        .unwrap();
    let gate_source = mosfet_design
        .add(parts::voltage_source("VG1", Real::from(2)))
        .unwrap();
    let load = mosfet_design
        .add(parts::resistor("R1", Real::one()))
        .unwrap();
    let transistor = mosfet_design
        .add(parts::nmos("M1", Real::one(), Real::one()))
        .unwrap();
    mosfet_design
        .connect(&vdd, [supply.pin("pos").unwrap(), load.pin("1").unwrap()])
        .unwrap();
    mosfet_design
        .connect(
            &gate,
            [
                gate_source.pin("pos").unwrap(),
                transistor.pin("G").unwrap(),
            ],
        )
        .unwrap();
    mosfet_design
        .connect(
            &drain,
            [load.pin("2").unwrap(), transistor.pin("D").unwrap()],
        )
        .unwrap();
    mosfet_design
        .connect(
            &ground,
            [
                supply.pin("neg").unwrap(),
                gate_source.pin("neg").unwrap(),
                transistor.pin("S").unwrap(),
            ],
        )
        .unwrap();
    let mosfet_circuit = mosfet_design.finish().unwrap().circuit;
    assert_eq!(mosfet_circuit.square_law_mosfets().unwrap().len(), 1);
    let mosfet_report = mosfet_circuit
        .solve_mosfet_dc(&MosfetNewtonPolicy::default(), &BTreeMap::new())
        .unwrap();
    assert_eq!(mosfet_report.status, MosfetNewtonStatus::Converged);
    assert!(mosfet_report.replay.exact_zero);
    assert_eq!(
        mosfet_report
            .net_voltage(&NetId::new("DRAIN").unwrap())
            .unwrap(),
        &(Real::from(9) / Real::from(2)).unwrap()
    );
}

#[test]
fn fluent_source_waveforms_drive_exact_transient_runs_and_reject_misuse() {
    let mut design = Design::new(
        "fluent-transient",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap()
    .transient_policy(TransientPolicy::GearBdf { order: 1 })
    .adapter_policy(hypercircuit::AdapterKind::TransientDae);
    let output = design.signal("OUT").unwrap();
    let ground = design.ground("GND").unwrap();
    let source = design
        .add(
            parts::current_source("S1", Real::one()).waveform(SourceWaveform::PiecewiseLinear {
                points: vec![
                    SourceWaveformPoint {
                        time: Real::zero(),
                        value: Real::zero(),
                    },
                    SourceWaveformPoint {
                        time: Real::from(2),
                        value: Real::from(2),
                    },
                    SourceWaveformPoint {
                        time: Real::from(3),
                        value: Real::from(2),
                    },
                ],
            }),
        )
        .unwrap();
    let capacitor = design.add(parts::capacitor("C1", Real::one())).unwrap();
    design
        .connect(
            &ground,
            [source.pin("pos").unwrap(), capacitor.pin("2").unwrap()],
        )
        .unwrap();
    design
        .connect(
            &output,
            [source.pin("neg").unwrap(), capacitor.pin("1").unwrap()],
        )
        .unwrap();
    assert!(matches!(
        design.stimulus(&source, SourceWaveform::Constant(Real::one())),
        Err(DesignBuildError::DuplicateStimulus(instance)) if instance == "S1"
    ));
    assert!(matches!(
        design.stimulus(
            &capacitor,
            SourceWaveform::Constant(Real::one())
        ),
        Err(DesignBuildError::InvalidStimulusTarget(instance)) if instance == "C1"
    ));
    assert_eq!(design.circuit().source_stimuli.len(), 1);

    let checked = design.finish().unwrap();
    let report = checked
        .circuit
        .transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::from(3),
                initial_timestep: Real::one(),
                minimum_timestep: Real::one(),
                maximum_timestep: Real::one(),
                maximum_accepted_steps: 3,
                maximum_rejected_steps: 1,
                adaptation: TransientAdaptation::Fixed,
            },
            Default::default(),
        )
        .unwrap();
    assert_eq!(report.status, TransientRunStatus::Complete);
    assert_eq!(
        report
            .source_waveform(&hypercircuit::ComponentId::new("S1").unwrap())
            .unwrap()
            .iter()
            .map(|(_, value)| value.clone())
            .collect::<Vec<_>>(),
        vec![Real::one(), Real::from(2), Real::from(2)]
    );
    assert_eq!(
        report
            .unknown_waveform(&MnaUnknown::NetVoltage(output.id().clone()))
            .unwrap()
            .iter()
            .map(|(_, value)| value.clone())
            .collect::<Vec<_>>(),
        vec![Real::one(), Real::from(3), Real::from(5)]
    );

    let mut invalid = Design::new(
        "invalid-stimulus",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let source = invalid
        .add(parts::voltage_source("V1", Real::zero()))
        .unwrap();
    let invalid_waveform = SourceWaveform::PiecewiseLinear { points: Vec::new() };
    let source_line = line!() + 1;
    invalid.stimulus(&source, invalid_waveform).unwrap();
    let report = invalid.check();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        matches!(
            &diagnostic.issue,
            DesignValidationIssue::Circuit(
                CircuitValidationIssue::EmptySourceWaveform(component)
            ) if component.as_str() == "V1"
        ) && diagnostic
            .sources
            .iter()
            .any(|source| source.file.ends_with("tests/authoring.rs") && source.line == source_line)
    }));

    let rejected =
        parts::resistor("R1", Real::one()).waveform(SourceWaveform::Constant(Real::one()));
    assert!(matches!(
        invalid.add(rejected),
        Err(DesignBuildError::InvalidStimulusTarget(instance)) if instance == "R1"
    ));
    assert!(
        invalid
            .circuit()
            .instances
            .iter()
            .all(|instance| instance.id.as_str() != "R1")
    );
}

#[test]
fn typed_buses_slices_and_ports_preserve_order_and_scope() {
    let mut design = Design::new(
        "fluent-interface",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let d0 = design.signal("D0").unwrap();
    let d1 = design.signal("D1").unwrap();
    let d2 = design.signal("D2").unwrap();
    let d3 = design.signal("D3").unwrap();
    let bus_line = line!() + 1;
    let data = design.bus("DATA", [&d0, &d1, &d2, &d3]).unwrap();
    let middle = design
        .bus_slice("MIDDLE", &data, 1, 2, BusSliceOrder::Reverse)
        .unwrap();
    let input = design
        .port("DATA_IN", &d0, PortDirection::Input, false)
        .unwrap();
    assert_eq!(
        data.members()
            .iter()
            .map(|net| net.id().as_str())
            .collect::<Vec<_>>(),
        vec!["D0", "D1", "D2", "D3"]
    );
    assert_eq!(
        middle
            .members()
            .iter()
            .map(|net| net.id().as_str())
            .collect::<Vec<_>>(),
        vec!["D2", "D1"]
    );
    assert_eq!(input.net().id(), d0.id());
    assert!(matches!(
        design.bus("DATA", [&d0]),
        Err(DesignBuildError::DuplicateBus(bus)) if bus == "DATA"
    ));
    assert!(matches!(
        design.bus("EMPTY", std::iter::empty()),
        Err(DesignBuildError::EmptyBus(bus)) if bus == "EMPTY"
    ));
    assert!(matches!(
        design.bus_slice("BAD", &data, 4, 1, BusSliceOrder::Forward),
        Err(DesignBuildError::InvalidBusSlice(slice)) if slice == "BAD"
    ));
    assert_eq!(design.circuit().buses.len(), 1);
    assert_eq!(design.circuit().bus_slices.len(), 1);
    assert_eq!(design.circuit().ports.len(), 1);
    assert!(design.check().is_valid());

    design.circuit_mut().buses[0].nets.clear();
    let report = design.check();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        matches!(
            &diagnostic.issue,
            DesignValidationIssue::Circuit(CircuitValidationIssue::EmptyBus(bus))
                if bus.as_str() == "DATA"
        ) && diagnostic
            .sources
            .iter()
            .any(|source| source.file.ends_with("tests/authoring.rs") && source.line == bus_line)
    }));

    let mut other = Design::new(
        "fluent-interface",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let foreign = other.signal("D0").unwrap();
    assert_eq!(
        design.bus("FOREIGN", [&foreign]),
        Err(DesignBuildError::ForeignHandle)
    );
    assert_eq!(
        design.port("FOREIGN", &foreign, PortDirection::Input, false),
        Err(DesignBuildError::ForeignHandle)
    );
}

#[test]
fn fluent_routing_vias_zones_and_keepouts_lower_atomically_with_sources() {
    let mut design = Design::new(
        "fluent-copper",
        BoardOutline::rectangle(Real::from(20), Real::from(12)),
        PcbStackup::two_layer(
            Real::one(),
            Real::one(),
            Some("copper".into()),
            Some("dielectric".into()),
        ),
    )
    .unwrap();
    let signal = design.signal("SIGNAL").unwrap();
    let route_line = line!() + 4;
    let route = design
        .route(
            &signal,
            Route::new("signal-route", TraceLayer(0), Real::one()).line(point(1, 2), point(8, 2)),
        )
        .unwrap();
    let via = design
        .via(
            &signal,
            Via::new(
                "signal-via",
                TraceLayer(0),
                TraceLayer(1),
                point(8, 2),
                Real::from(2),
                Real::one(),
            )
            .mask(ViaMaskIntent::tented()),
        )
        .unwrap();
    let zone = design
        .zone(
            &signal,
            Zone::solid(
                "signal-zone",
                TraceLayer(1),
                vec![point(1, 1), point(10, 1), point(10, 5), point(1, 5)],
            )
            .clearance(Real::one())
            .priority(2),
        )
        .unwrap();
    let keepout = design
        .keepout(Keepout::new(
            "connector-clearance",
            vec![point(12, 1), point(18, 1), point(18, 5), point(12, 5)],
            KeepoutScope::Components,
        ))
        .unwrap();
    assert!(route.belongs_to(&design));
    assert!(via.belongs_to(&design));
    assert!(zone.belongs_to(&design));
    assert!(keepout.belongs_to(&design));
    assert_eq!(route.id().as_str(), "signal-route");
    assert_eq!(via.id().as_str(), "signal-via");
    assert_eq!(zone.id().as_str(), "signal-zone");
    assert_eq!(keepout.id().as_str(), "connector-clearance");

    assert!(matches!(
        design.route(
            &signal,
            Route::new("signal-route", TraceLayer(0), Real::one())
                .line(point(1, 3), point(8, 3))
        ),
        Err(DesignBuildError::DuplicateRoute(route)) if route == "signal-route"
    ));
    assert!(matches!(
        design.route(
            &signal,
            Route::new("broken", TraceLayer(0), Real::one())
                .line(point(1, 2), point(3, 2))
                .line(point(4, 2), point(8, 2))
        ),
        Err(DesignBuildError::InvalidRoute(route)) if route == "broken"
    ));
    assert!(matches!(
        design.route(
            &signal,
            Route::new("missing-layer", TraceLayer(2), Real::one())
                .line(point(1, 2), point(8, 2))
        ),
        Err(DesignBuildError::InvalidRoute(route)) if route == "missing-layer"
    ));
    assert!(matches!(
        design.via(
            &signal,
            Via::new(
                "broken",
                TraceLayer(0),
                TraceLayer(1),
                point(8, 2),
                Real::one(),
                Real::from(2),
            )
        ),
        Err(DesignBuildError::InvalidVia(via)) if via == "broken"
    ));
    assert!(matches!(
        design.via(
            &signal,
            Via::new(
                "missing-layer",
                TraceLayer(0),
                TraceLayer(2),
                point(8, 2),
                Real::from(2),
                Real::one(),
            )
        ),
        Err(DesignBuildError::InvalidVia(via)) if via == "missing-layer"
    ));
    assert!(matches!(
        design.zone(
            &signal,
            Zone::solid(
                "broken",
                TraceLayer(0),
                vec![point(0, 0), point(1, 0)]
            )
        ),
        Err(DesignBuildError::InvalidZone(zone)) if zone == "broken"
    ));
    assert!(matches!(
        design.zone(
            &signal,
            Zone::solid(
                "missing-layer",
                TraceLayer(2),
                vec![point(1, 1), point(10, 1), point(10, 5), point(1, 5)]
            )
        ),
        Err(DesignBuildError::InvalidZone(zone)) if zone == "missing-layer"
    ));
    assert!(matches!(
        design.keepout(Keepout::new(
            "broken",
            vec![point(0, 0), point(1, 0)],
            KeepoutScope::All,
        )),
        Err(DesignBuildError::InvalidKeepout(keepout)) if keepout == "broken"
    ));
    assert_eq!(design.layout().routes.len(), 1);
    assert_eq!(design.layout().vias.len(), 1);
    assert_eq!(design.layout().zones.len(), 1);
    assert_eq!(design.layout().keepouts.len(), 1);

    let mut other = Design::new(
        "fluent-copper",
        BoardOutline::rectangle(Real::from(20), Real::from(12)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let foreign = other.signal("SIGNAL").unwrap();
    assert_eq!(
        design.route(
            &foreign,
            Route::new("foreign", TraceLayer(0), Real::one()).line(point(1, 2), point(8, 2))
        ),
        Err(DesignBuildError::ForeignHandle)
    );
    assert_eq!(design.layout().routes.len(), 1);

    design.layout_mut().routes[0].width = Real::zero();
    let report = design.check();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        matches!(
            &diagnostic.issue,
            DesignValidationIssue::Layout(
                hypercircuit::LayoutValidationIssue::NonPositiveRouteWidth(route)
            ) if route.as_str() == "signal-route"
        ) && diagnostic
            .sources
            .iter()
            .any(|source| source.file.ends_with("tests/authoring.rs") && source.line == route_line)
    }));
    design.layout_mut().routes[0].width = Real::one();

    let checked = design.finish().unwrap();
    assert_eq!(checked.layout.routes[0].net.as_str(), "SIGNAL");
    assert_eq!(checked.layout.vias[0].net.as_str(), "SIGNAL");
    assert_eq!(checked.layout.zones[0].net.as_str(), "SIGNAL");
    assert!(checked.source_map.traces.iter().any(|trace| {
        trace.action == AuthoringAction::Declare
            && matches!(
                &trace.target,
                AuthoringTarget::Route(route) if route.as_str() == "signal-route"
            )
    }));
    #[cfg(feature = "interchange")]
    {
        let encoded = serde_json::to_string(&checked.source_map).unwrap();
        let decoded: hypercircuit::DesignSourceMap = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, checked.source_map);
    }
}

#[test]
fn typed_placement_rules_cover_every_retained_constraint_family_atomically() {
    let mut design = Design::new(
        "fluent-placement-rules",
        BoardOutline::rectangle(Real::from(20), Real::from(12)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let common = design.ground("GND").unwrap();
    let footprint =
        || Footprint::two_pad_smd(Real::one(), Real::one(), Real::from(2), vec![TraceLayer(0)]);
    let first = design
        .add(
            parts::resistor("R1", Real::from(1_000))
                .footprint(footprint())
                .at(point(1, 1)),
        )
        .unwrap();
    let second = design
        .add(
            parts::resistor("R2", Real::from(1_000))
                .footprint(footprint())
                .at(point(4, 4)),
        )
        .unwrap();
    design
        .connect(
            &common,
            [
                first.pin("1").unwrap(),
                first.pin("2").unwrap(),
                second.pin("1").unwrap(),
                second.pin("2").unwrap(),
            ],
        )
        .unwrap();

    let fixed = design
        .constrain(PlacementRule::fixed("r1-fixed", &first, point(2, 2)))
        .unwrap();
    design
        .constrain(PlacementRule::relative(
            "r2-relative",
            &second,
            &first,
            point(0, 0),
        ))
        .unwrap();
    design
        .constrain(PlacementRule::align_x("same-x", [&first, &second]))
        .unwrap();
    design
        .constrain(PlacementRule::align_y("same-y", [&first, &second]))
        .unwrap();
    design
        .constrain(PlacementRule::within(
            "r1-region",
            &first,
            point(0, 0),
            point(10, 10),
        ))
        .unwrap();
    design
        .constrain(PlacementRule::allowed_rotations(
            "r1-rotation",
            &first,
            vec![Real::zero()],
        ))
        .unwrap();
    design
        .constrain(PlacementRule::allowed_sides(
            "r1-side",
            &first,
            vec![BoardSide::Front],
        ))
        .unwrap();
    assert!(fixed.belongs_to(&design));
    assert_eq!(fixed.id().as_str(), "r1-fixed");
    assert_eq!(design.layout().placement_constraints.len(), 7);
    let resolved = design
        .layout()
        .resolve_placement_constraints(design.circuit());
    assert!(resolved.is_satisfied());
    assert!(
        resolved
            .placements
            .iter()
            .all(|placement| placement.position == point(2, 2))
    );

    assert!(matches!(
        design.constrain(PlacementRule::within(
            "r1-region",
            &first,
            point(0, 0),
            point(10, 10),
        )),
        Err(DesignBuildError::DuplicatePlacementConstraint(constraint))
            if constraint == "r1-region"
    ));
    assert!(matches!(
        design.constrain(PlacementRule::align_x("invalid-arity", [&first])),
        Err(DesignBuildError::InvalidPlacementConstraint(constraint))
            if constraint == "invalid-arity"
    ));
    assert!(matches!(
        design.constrain(PlacementRule::fixed(
            "second-driver",
            &second,
            point(3, 3)
        )),
        Err(DesignBuildError::InvalidPlacementConstraint(constraint))
            if constraint == "second-driver"
    ));
    assert_eq!(design.layout().placement_constraints.len(), 7);

    let mut other = Design::new(
        "fluent-placement-rules",
        BoardOutline::rectangle(Real::from(20), Real::from(12)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let foreign = other
        .add(
            parts::resistor("R1", Real::one())
                .footprint(footprint())
                .at(point(1, 1)),
        )
        .unwrap();
    assert_eq!(
        design.constrain(PlacementRule::fixed("foreign", &foreign, point(1, 1))),
        Err(DesignBuildError::ForeignHandle)
    );

    let checked = design.finish().unwrap();
    assert!(checked.source_map.traces.iter().any(|trace| {
        matches!(
            &trace.target,
            AuthoringTarget::PlacementConstraint(constraint)
                if constraint.as_str() == "r1-fixed"
        ) && trace.source.file.ends_with("tests/authoring.rs")
    }));
}

#[test]
fn typed_routing_policy_builders_retain_inheritance_vias_and_pairs_atomically() {
    let mut design = Design::new(
        "fluent-routing-policy",
        BoardOutline::rectangle(Real::from(20), Real::from(12)),
        PcbStackup::two_layer(
            Real::one(),
            Real::one(),
            Some("copper".into()),
            Some("dielectric".into()),
        ),
    )
    .unwrap();
    let positive = design.signal("USB_D+").unwrap();
    let negative = design.signal("USB_D-").unwrap();
    let _ground = design.ground("GND").unwrap();

    let style = design
        .via_style(
            ViaStyleRule::new("through", Real::from(2), Real::one())
                .mask(ViaMaskIntent::tented())
                .span(TraceLayer(0), TraceLayer(1)),
        )
        .unwrap();
    let base = design
        .net_class(
            NetClassRule::new("controlled", std::iter::empty::<&NetHandle>())
                .min_trace_width(Real::one())
                .preferred_trace_width(Real::from(2))
                .min_clearance(Real::one())
                .preferred_via_geometry(Real::from(2), Real::one())
                .preferred_via_style(&style)
                .max_length(Real::from(100))
                .max_via_count(2)
                .reference_plane(),
        )
        .unwrap();
    let signals = design
        .net_class(NetClassRule::new("usb", [&positive, &negative]).parent(&base))
        .unwrap();
    let pair = design
        .differential_pair(
            DifferentialPairRule::new("usb-data", &positive, &negative, Real::one())
                .max_skew(Real::one())
                .impedance(Real::from(90), Real::from(10))
                .neckdown(
                    (Real::one() / Real::from(2)).unwrap(),
                    (Real::one() / Real::from(2)).unwrap(),
                    Real::from(2),
                ),
        )
        .unwrap();
    assert!(style.belongs_to(&design));
    assert!(base.belongs_to(&design));
    assert!(signals.belongs_to(&design));
    assert!(pair.belongs_to(&design));
    assert_eq!(style.id().as_str(), "through");
    assert_eq!(signals.id().as_str(), "usb");
    assert_eq!(pair.id().as_str(), "usb-data");
    assert_eq!(
        design.layout().rules.differential_pairs[0].target_impedance_ohms,
        Some(Real::from(90))
    );
    assert_eq!(
        design.layout().rules.differential_pairs[0]
            .neckdown
            .as_ref()
            .unwrap()
            .maximum_transition_length,
        Real::from(2)
    );
    assert_eq!(
        design.layout().rules.differential_pairs[0]
            .neckdown
            .as_ref()
            .unwrap()
            .trace_width,
        (Real::one() / Real::from(2)).unwrap()
    );

    let resolved = design.layout().rules.resolve_net_classes().unwrap();
    let usb = resolved
        .iter()
        .find(|class| class.id.as_str() == "usb")
        .unwrap();
    assert_eq!(usb.lineage.len(), 2);
    assert_eq!(usb.min_trace_width, Some(Real::one()));
    assert_eq!(
        usb.preferred_via_style.as_ref().unwrap().as_str(),
        "through"
    );
    assert_eq!(usb.target_impedance_ohms, None);
    assert_eq!(usb.impedance_tolerance_ohms, None);
    assert!(usb.requires_reference_plane);

    assert!(matches!(
        design.via_style(ViaStyleRule::new(
            "through",
            Real::from(2),
            Real::one()
        )),
        Err(DesignBuildError::DuplicateViaStyle(style)) if style == "through"
    ));
    assert!(matches!(
        design.via_style(
            ViaStyleRule::new("missing-layer", Real::from(2), Real::one())
                .span(TraceLayer(0), TraceLayer(2))
        ),
        Err(DesignBuildError::InvalidViaStyle(style)) if style == "missing-layer"
    ));
    assert!(matches!(
        design.net_class(NetClassRule::new("duplicate-net", [&positive, &positive])),
        Err(DesignBuildError::InvalidNetClass(class)) if class == "duplicate-net"
    ));
    assert!(matches!(
        design.differential_pair(DifferentialPairRule::new(
            "same-net",
            &positive,
            &positive,
            Real::one(),
        )),
        Err(DesignBuildError::InvalidDifferentialPair(pair)) if pair == "same-net"
    ));
    assert!(matches!(
        design.differential_pair(
            DifferentialPairRule::new("invalid-neckdown", &positive, &negative, Real::one())
                .neckdown(Real::one(), Real::one(), Real::one())
        ),
        Err(DesignBuildError::InvalidDifferentialPair(pair)) if pair == "invalid-neckdown"
    ));
    assert_eq!(design.layout().rules.via_styles.len(), 1);
    assert_eq!(design.layout().rules.net_classes.len(), 2);
    assert_eq!(design.layout().rules.differential_pairs.len(), 1);

    let mut other = Design::new(
        "fluent-routing-policy",
        BoardOutline::rectangle(Real::from(20), Real::from(12)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let foreign = other.signal("USB_D+").unwrap();
    assert_eq!(
        design.net_class(NetClassRule::new("foreign", [&foreign])),
        Err(DesignBuildError::ForeignHandle)
    );

    design.layout_mut().rules.net_classes[1].min_trace_width = Some(Real::zero());
    let report = design.check();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        matches!(
            &diagnostic.issue,
            DesignValidationIssue::Layout(
                hypercircuit::LayoutValidationIssue::InvalidNetClassConstraint(class)
            ) if class.as_str() == "usb"
        ) && diagnostic
            .sources
            .iter()
            .any(|source| source.file.ends_with("tests/authoring.rs"))
    }));
    design.layout_mut().rules.net_classes[1].min_trace_width = None;

    let checked = design.finish().unwrap();
    assert!(checked.source_map.traces.iter().any(|trace| {
        matches!(
            &trace.target,
            AuthoringTarget::DifferentialPair(pair) if pair.as_str() == "usb-data"
        )
    }));
}

#[test]
fn typed_phase_tuning_builders_retain_and_realize_one_atomic_pair() {
    let mut design = Design::new(
        "fluent-phase-tuning",
        BoardOutline::rectangle(Real::from(12), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let positive = design.signal("D+").unwrap();
    let negative = design.signal("D-").unwrap();
    let pair = design
        .differential_pair(
            DifferentialPairRule::new("data", &positive, &negative, Real::one())
                .max_skew(Real::one()),
        )
        .unwrap();
    let positive_route = design
        .route(
            &positive,
            Route::new("data-p", TraceLayer(0), Real::one()).line(point(2, 4), point(8, 4)),
        )
        .unwrap();
    let negative_route = design
        .route(
            &negative,
            Route::new("data-n", TraceLayer(0), Real::one()).line(point(2, 6), point(8, 6)),
        )
        .unwrap();
    let positive_tuning = design
        .length_tuning(
            LengthTuningRule::new(
                "data-p-tuning",
                &positive,
                vec![point(1, 3), point(9, 3), point(9, 6), point(1, 6)],
                Real::from(10),
                Real::one(),
                Real::one(),
                2,
            )
            .route(&positive_route),
        )
        .unwrap();
    let negative_tuning = design
        .length_tuning(
            LengthTuningRule::new(
                "data-n-tuning",
                &negative,
                vec![point(1, 5), point(9, 5), point(9, 8), point(1, 8)],
                Real::from(10),
                Real::one(),
                Real::one(),
                2,
            )
            .route(&negative_route),
        )
        .unwrap();
    let group = design
        .phase_tuning_group(
            PhaseTuningGroupRule::new(
                "data-phase",
                [positive_tuning.clone(), negative_tuning.clone()],
            )
            .differential_pair(&pair),
        )
        .unwrap();

    assert!(positive_tuning.belongs_to(&design));
    assert!(group.belongs_to(&design));
    let report = design
        .layout()
        .realize_phase_tuning(design.circuit(), group.id());
    assert_eq!(report.status, PhaseTuningStatus::Applied);
    assert_eq!(report.tuned_routes.len(), 2);
    assert_eq!(report.realized_skew, Some(Real::zero()));
    assert!(design.source_map().traces.iter().any(|trace| {
        matches!(
            &trace.target,
            AuthoringTarget::PhaseTuningGroup(id) if id.as_str() == "data-phase"
        )
    }));
    assert!(design.finish().is_ok());
}
