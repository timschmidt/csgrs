use hyperparts::{
    AbsoluteMaximumRating, AspectKind, Assertion, AssertionCondition, AssertionConfidence,
    AssertionValue, AutorouterOutputRecord, CalibrationState, Capability, CapabilityEnvelope,
    CapabilityInput, CapabilityOutput, CapabilityStatus, CircuitJsonSourceRecord,
    CompatibilityClass, CompatibilityKind, CompatibilityRelation, ComplianceClaim,
    ConnectionDecision, ConsumableRequirement, EdaAuthoringBundle, EdaExactField,
    EdaFabricationReadiness, EdaFootprintString, EdaHandoffStatus, EdaIntakeStatus, EdaModelStatus,
    EdaPackageMetadata, EdaPackagePin, EdaRouteStatus, ElectricalCompatibilityReport,
    ElectricalFactStatus, ElectricalPolarity, ElectronicPackage, FabricationOutputRecord,
    FixtureRequirement, GeneralPartAssertion, GeneratedModelReference, GeometryHandle,
    GeometryHandoffReport, GeometryStatus, GridFeature, GridSystem, ImportIssue, ImportIssueKind,
    ImportReport, ImportTargetKind, InteractionKind, Interface, InterfaceKind,
    ManufacturerPartNumber, ManufacturingRoute, MassPropertyNeed, MaterialRequirement,
    MechanicalLoadPath, MountingFeature, MountingPattern, Operation, PartAspect, PartAssertion,
    PartConstraint, PartFamily, PartGraph, PartId, PartKnowledgeReport, PartQuery,
    PartQueryEvidence, PhysicalFactStatus, PhysicalPropertyHandle, PhysicsHandoffReport,
    PinFunction, PinMap, Pinout, PowerDomain, PowerIntent, Process, ProcessCapability, ProcessKind,
    ProcurementOffer, Real, RecommendedOperatingCondition, ReferenceDesignatorClass, Relationship,
    RelationshipKind, SafeConnectionReport, ShapeSource, SourceRef, SourceRevision, SupplierSku,
    SupplyRail, Terminal, TerminalId, TerminalRole, ThermalPath, ToleranceEnvelope, ToolCapability,
    ToolPart, VariantId, VoltageEnvelope, VoltageRange, import_eda_authoring_bundle,
};
use proptest::prelude::*;

fn source() -> SourceRef {
    SourceRef::new("fixture", "library").unwrap()
}

fn part(id: &str) -> PartId {
    PartId::new(id).unwrap()
}

fn variant(id: &str) -> VariantId {
    VariantId::new(id).unwrap()
}

fn terminal(id: &str) -> TerminalId {
    TerminalId::new(id).unwrap()
}

fn voltage(min: i32, max: i32) -> VoltageEnvelope {
    VoltageEnvelope::new(Real::from(min), Real::from(max)).unwrap()
}

#[test]
fn kicad_style_symbol_footprint_package_device_mapping_keeps_evidence() {
    let mut graph = PartGraph::default();
    let resistor_symbol = part("kicad:symbol:R");
    let resistor_footprint = part("kicad:footprint:R_0603");
    let package = part("package:0603");
    let device = part("device:rc0603fr");

    for id in [&resistor_symbol, &resistor_footprint, &package, &device] {
        graph.insert_family(PartFamily::new(id.clone(), id.as_str()));
    }

    let evidence = PartQueryEvidence::from_fact(source(), "symbol-footprint-package-device");
    graph.add_compatibility(CompatibilityRelation {
        left: resistor_symbol.clone(),
        right: resistor_footprint.clone(),
        kind: CompatibilityKind::SymbolFootprint,
        evidence: evidence.clone(),
    });
    graph.add_compatibility(CompatibilityRelation {
        left: resistor_footprint,
        right: package.clone(),
        kind: CompatibilityKind::FootprintPackage,
        evidence: evidence.clone(),
    });
    graph.add_compatibility(CompatibilityRelation {
        left: package,
        right: device,
        kind: CompatibilityKind::PackageDevice,
        evidence,
    });

    assert_eq!(graph.compatibility().len(), 3);
    assert_eq!(
        graph.compatibility()[0].evidence.facts,
        vec!["symbol-footprint-package-device"]
    );
}

#[test]
fn safe_connection_uses_polarity_and_exact_voltage_envelopes() {
    let mut graph = PartGraph::default();
    let board = part("board");
    let rev = variant("A");
    let mut family = PartFamily::new(board.clone(), "Board");
    let mut board_variant = hyperparts::PartVariant::new(rev.clone(), None);
    board_variant.add_terminal(Terminal::new(
        terminal("5v"),
        "5V",
        ElectricalPolarity::Power,
        Some(voltage(5, 5)),
    ));
    board_variant.add_terminal(Terminal::new(
        terminal("3v3"),
        "3V3",
        ElectricalPolarity::Power,
        Some(voltage(3, 3)),
    ));
    board_variant.add_terminal(Terminal::new(
        terminal("gnd"),
        "GND",
        ElectricalPolarity::Ground,
        None,
    ));
    board_variant.add_terminal(Terminal::new(
        terminal("sda"),
        "SDA",
        ElectricalPolarity::Signal,
        None,
    ));
    board_variant.add_terminal(Terminal::new(
        terminal("scl"),
        "SCL",
        ElectricalPolarity::Signal,
        None,
    ));
    board_variant.add_interface(Interface::new(
        "i2c",
        InterfaceKind::Electrical,
        vec![terminal("sda"), terminal("scl")],
    ));
    family.insert_variant(board_variant);
    graph.insert_family(family);

    let safe_signal = graph
        .safe_connection(
            (&board, &rev, &terminal("sda")),
            (&board, &rev, &terminal("scl")),
        )
        .unwrap();
    assert_eq!(safe_signal.value, ConnectionDecision::Safe);
    assert!(!safe_signal.evidence.facts.is_empty());

    let unsafe_power_ground = graph
        .safe_connection(
            (&board, &rev, &terminal("5v")),
            (&board, &rev, &terminal("gnd")),
        )
        .unwrap();
    assert_eq!(unsafe_power_ground.value, ConnectionDecision::Unsafe);

    let mismatched_rails = graph
        .safe_connection(
            (&board, &rev, &terminal("5v")),
            (&board, &rev, &terminal("3v3")),
        )
        .unwrap();
    assert_eq!(mismatched_rails.value, ConnectionDecision::Unsafe);
}

#[test]
fn unknown_import_fields_stay_explicit() {
    let mut variant = hyperparts::PartVariant::new(variant("gridbeam-1in"), None);
    variant.add_assertion(PartAssertion::ShapeHandle(Assertion::unknown(source())));
    variant.add_process(ProcessCapability {
        kind: ProcessKind::Subtractive,
        statement: "drilled bolting grid".into(),
    });

    assert!(matches!(
        &variant.assertions()[0],
        PartAssertion::ShapeHandle(assertion) if assertion.value().is_none()
    ));

    let mut graph = PartGraph::default();
    graph.add_tool(ToolCapability {
        tool: "manual-drill".into(),
        capability: ProcessCapability {
            kind: ProcessKind::Inspection,
            statement: "hole spacing check".into(),
        },
    });
    graph.add_import_report(ImportReport {
        source: source(),
        target: ImportTargetKind::KiCad,
        imported_family_count: 1,
        imported_variant_count: 1,
        unknown_field_count: 1,
        parsed_assertions: Vec::new(),
        rejected_fields: Vec::new(),
        lossy_conversions: Vec::new(),
        stale_sources: Vec::new(),
        unresolved_references: Vec::new(),
        license_notes: Vec::new(),
        review_requirements: Vec::new(),
        warnings: vec!["shape handle omitted by fixture".into()],
    });
    assert_eq!(graph.import_reports()[0].unknown_field_count, 1);
    assert_eq!(graph.tools().len(), 1);
}

#[test]
fn importer_reports_keep_rejected_lossy_stale_and_review_fields_structured() {
    let report = ImportReport {
        source: source(),
        target: ImportTargetKind::NopScadLib,
        imported_family_count: 2,
        imported_variant_count: 3,
        unknown_field_count: 4,
        parsed_assertions: vec![ImportIssue {
            field: "vitamins/resistors".into(),
            kind: ImportIssueKind::Parsed,
            detail: "parsed resistor family table".into(),
        }],
        rejected_fields: vec![ImportIssue {
            field: "module/rendered_preview".into(),
            kind: ImportIssueKind::Rejected,
            detail: "preview is display-only geometry".into(),
        }],
        lossy_conversions: vec![ImportIssue {
            field: "stl/body".into(),
            kind: ImportIssueKind::LossyConversion,
            detail: "mesh retained as lossy artifact".into(),
        }],
        stale_sources: vec![ImportIssue {
            field: "distributor/stock".into(),
            kind: ImportIssueKind::StaleSource,
            detail: "snapshot older than review policy".into(),
        }],
        unresolved_references: vec![ImportIssue {
            field: "footprint:missing".into(),
            kind: ImportIssueKind::UnresolvedReference,
            detail: "footprint library not available".into(),
        }],
        license_notes: vec![ImportIssue {
            field: "source/license".into(),
            kind: ImportIssueKind::LicenseNote,
            detail: "manual review required before redistribution".into(),
        }],
        review_requirements: vec![ImportIssue {
            field: "pin-map".into(),
            kind: ImportIssueKind::ReviewRequired,
            detail: "source omitted package orientation".into(),
        }],
        warnings: vec!["adapter kept all uncertain fields explicit".into()],
    };

    assert_eq!(report.target, ImportTargetKind::NopScadLib);
    assert_eq!(
        report.lossy_conversions[0].kind,
        ImportIssueKind::LossyConversion
    );
    assert_eq!(report.review_requirements.len(), 1);
    assert_eq!(report.unknown_field_count, 4);
}

#[test]
fn general_assertions_preserve_ranges_conditions_confidence_and_revision() {
    let src = source();
    let value = AssertionValue::interval(Real::from(5), Real::from(10)).unwrap();
    let assertion = PartAssertion::General(Box::new(GeneralPartAssertion {
        key: "operating-temperature".into(),
        value: value.clone(),
        unit: Some("degC".into()),
        conditions: vec![AssertionCondition {
            label: "airflow".into(),
            value: AssertionValue::Text("still-air".into()),
        }],
        confidence: AssertionConfidence::Reviewed,
        source: src.clone(),
        revision: Some(SourceRevision::new("rev-a", Some("2026-05-18".into())).unwrap()),
    }));

    assert_eq!(
        AssertionValue::interval(Real::from(10), Real::from(5)),
        Err(hyperparts::PartsError::InvalidAssertionRange)
    );
    match assertion {
        PartAssertion::General(general) => {
            assert_eq!(general.value, value);
            assert_eq!(general.conditions.len(), 1);
            assert_eq!(general.confidence, AssertionConfidence::Reviewed);
            assert_eq!(general.source, src);
            assert_eq!(general.revision.unwrap().revision, "rev-a");
        }
        _ => panic!("expected general assertion"),
    }
}

#[test]
fn procurement_compliance_and_knowledge_reports_keep_unknowns_visible() {
    let offer = ProcurementOffer {
        sku: SupplierSku::new("DIGI-123").unwrap(),
        mpn: Some(ManufacturerPartNumber::new("MFG-456").unwrap()),
        quantity: AssertionValue::exact_scalar(Real::from(100)),
        unit_price: AssertionValue::Lossy("0.0123 USD from distributor snapshot".into()),
        currency: Some("USD".into()),
        source: source(),
    };
    let compliance = ComplianceClaim {
        scheme: "RoHS".into(),
        value: AssertionValue::Unknown,
        source: source(),
    };
    let report = PartKnowledgeReport {
        status: "needs-review".into(),
        evidence: vec!["supplier snapshot imported".into()],
        unknowns: vec!["RoHS source table omitted value".into()],
        conflicts: Vec::new(),
    };

    assert_eq!(
        offer.quantity,
        AssertionValue::exact_scalar(Real::from(100))
    );
    assert_eq!(compliance.value, AssertionValue::Unknown);
    assert_eq!(report.unknowns.len(), 1);
}

#[test]
fn aspects_relationships_and_feature_roles_are_part_graph_facts() {
    let mut graph = PartGraph::default();
    let beam = part("gridbeam:beam");
    let bolt = part("gridbeam:bolt");
    let mut family = PartFamily::new(beam.clone(), "Gridbeam");
    let mut beam_variant = hyperparts::PartVariant::new(variant("1x4"), None);
    beam_variant.add_aspect(PartAspect::new("gridbeam:beam/body", AspectKind::Body));
    beam_variant.add_terminal(Terminal::new(
        terminal("hole-a"),
        "Hole A",
        ElectricalPolarity::Passive,
        None,
    ));
    family.insert_variant(beam_variant);
    graph.insert_family(family);
    graph.insert_family(PartFamily::new(bolt, "Bolt"));

    graph.add_relationship(Relationship {
        left: "gridbeam:beam/hole-a".into(),
        right: "gridbeam:bolt/thread".into(),
        kind: RelationshipKind::Fastens,
        compatibility: CompatibilityClass::Certified,
        interaction: InteractionKind::Mechanical,
        evidence: PartQueryEvidence::from_fact(source(), "gridbeam hole accepts bolt"),
    });

    let feature = MountingFeature {
        handle: "gridbeam:beam/hole-a".into(),
        descriptor: "1in grid hole".into(),
    };

    assert_eq!(graph.relationships().len(), 1);
    assert_eq!(graph.relationships()[0].kind, RelationshipKind::Fastens);
    assert_eq!(
        graph
            .family(&beam)
            .unwrap()
            .variant(&variant("1x4"))
            .unwrap()
            .aspects()
            .len(),
        1
    );
    assert_eq!(feature.descriptor, "1in grid hole");
    assert_eq!(TerminalRole::Mounting, TerminalRole::Mounting);
    assert_eq!(ReferenceDesignatorClass::J, ReferenceDesignatorClass::J);
}

#[test]
fn geometry_handoff_reports_do_not_claim_ownership_or_certainty() {
    let grid = GridSystem {
        name: "gridbeam".into(),
        spacing: "1in".into(),
    };
    let pattern = MountingPattern {
        handle: "gridbeam:beam/pattern".into(),
        features: vec![GridFeature {
            handle: "gridbeam:beam/hole-a".into(),
            grid,
        }],
    };
    let report = GeometryHandoffReport {
        part: part("gridbeam:beam"),
        source: ShapeSource::Grid,
        geometry: Some(GeometryHandle {
            owner: "hypercurve".into(),
            handle: "profile:gridbeam-hole".into(),
            units: Some("inch".into()),
        }),
        status: GeometryStatus::Exact,
        evidence: PartQueryEvidence::from_fact(source(), "grid spacing imported from fixture"),
    };

    assert_eq!(pattern.features.len(), 1);
    assert_eq!(report.geometry.as_ref().unwrap().owner, "hypercurve");
    assert_eq!(report.status, GeometryStatus::Exact);

    let missing = GeometryHandoffReport {
        part: part("ic:closed"),
        source: ShapeSource::ModelArtifact,
        geometry: None,
        status: GeometryStatus::Missing,
        evidence: PartQueryEvidence::from_fact(source(), "datasheet did not include STEP model"),
    };
    assert_eq!(missing.status, GeometryStatus::Missing);
}

#[test]
fn physics_handoff_requires_explicit_status_and_keeps_unknowns_visible() {
    let density = PhysicalPropertyHandle::new(
        "hyperphysics",
        "material:al6061/density",
        "density",
        Some("kg/m^3".into()),
        PhysicalFactStatus::Certified,
    )
    .unwrap();
    let conductivity = MaterialRequirement {
        key: "thermal-conductivity".into(),
        value: AssertionValue::interval(Real::from(160), Real::from(170)).unwrap(),
        units: Some("W/(m*K)".into()),
        conditions: vec![AssertionCondition {
            label: "temperature".into(),
            value: AssertionValue::exact_scalar(Real::from(293)),
        }],
        status: PhysicalFactStatus::Conditional,
    };
    let report = PhysicsHandoffReport {
        part: part("heatsink:al6061"),
        handles: vec![density.clone()],
        requirements: vec![conductivity],
        environments: Vec::new(),
        thermal_paths: vec![ThermalPath {
            from: "package:top".into(),
            to: "heatsink:base".into(),
            property: Some(density),
            value: AssertionValue::Unknown,
            status: PhysicalFactStatus::Unknown,
        }],
        mechanical_load_paths: vec![MechanicalLoadPath {
            from: "bracket:slot".into(),
            to: "frame:rail".into(),
            direction: "normal".into(),
            load: AssertionValue::Lossy("datasheet says hand-tight".into()),
            status: PhysicalFactStatus::Lossy,
        }],
        mass_property_needs: vec![MassPropertyNeed {
            target: "heatsink:body".into(),
            material: None,
            outputs: vec!["mass".into(), "center_of_mass".into()],
            status: PhysicalFactStatus::Unknown,
        }],
        status: PhysicalFactStatus::Conditional,
        evidence: PartQueryEvidence::from_fact(source(), "thermal path imported from fixture"),
        unknowns: vec!["contact resistance omitted".into()],
    };

    assert_eq!(
        PhysicalPropertyHandle::new(
            "",
            "material:al6061/density",
            "density",
            Some("kg/m^3".into()),
            PhysicalFactStatus::Certified,
        ),
        Err(hyperparts::PartsError::EmptyIdentifier)
    );
    assert!(!report.is_certified_ready());
    assert_eq!(report.thermal_paths[0].value, AssertionValue::Unknown);
    assert_eq!(
        report.mechanical_load_paths[0].status,
        PhysicalFactStatus::Lossy
    );
}

#[test]
fn exact_physics_report_is_ready_only_when_requirements_are_certified() {
    let density = PhysicalPropertyHandle::new(
        "hyperphysics",
        "material:fr4/density",
        "density",
        Some("kg/m^3".into()),
        PhysicalFactStatus::Exact,
    )
    .unwrap();
    let report = PhysicsHandoffReport {
        part: part("pcb:coupon"),
        handles: vec![density.clone()],
        requirements: vec![MaterialRequirement {
            key: "density".into(),
            value: AssertionValue::exact_scalar(Real::from(1850)),
            units: Some("kg/m^3".into()),
            conditions: Vec::new(),
            status: PhysicalFactStatus::Certified,
        }],
        environments: Vec::new(),
        thermal_paths: Vec::new(),
        mechanical_load_paths: Vec::new(),
        mass_property_needs: vec![MassPropertyNeed {
            target: "pcb:coupon/body".into(),
            material: Some(density),
            outputs: vec!["mass".into()],
            status: PhysicalFactStatus::Certified,
        }],
        status: PhysicalFactStatus::Certified,
        evidence: PartQueryEvidence::from_fact(source(), "density reviewed from material card"),
        unknowns: Vec::new(),
    };

    assert!(report.is_certified_ready());
}

#[test]
fn electronics_report_keeps_closed_ic_internal_structure_unknown() {
    let vcc = terminal("vcc");
    let gnd = terminal("gnd");
    let rail = VoltageRange::new(Real::from(3), Real::from(5)).unwrap();
    let report = ElectricalCompatibilityReport {
        part: part("ic:closed-opamp"),
        package: Some(ElectronicPackage {
            name: "SOIC-8".into(),
            handle: "package:soic-8".into(),
            terminal_count: Some(8),
            status: ElectricalFactStatus::Certified,
        }),
        pinout: vec![
            Pinout {
                terminal: vcc.clone(),
                name: "VCC".into(),
                function: PinFunction::Power,
                voltage: Some(rail.clone()),
                current: None,
                status: ElectricalFactStatus::Certified,
            },
            Pinout {
                terminal: gnd.clone(),
                name: "GND".into(),
                function: PinFunction::Ground,
                voltage: Some(VoltageRange::new(Real::from(0), Real::from(0)).unwrap()),
                current: None,
                status: ElectricalFactStatus::Certified,
            },
        ],
        pin_maps: vec![PinMap {
            package_terminal: vcc.clone(),
            internal: AssertionValue::Unknown,
            status: ElectricalFactStatus::Unknown,
        }],
        die_ports: Vec::new(),
        die_nets: Vec::new(),
        power_intent: Some(PowerIntent {
            domains: vec![PowerDomain {
                name: "analog".into(),
                rails: vec![SupplyRail {
                    name: "VCC".into(),
                    voltage: rail,
                    terminals: vec![vcc],
                    status: ElectricalFactStatus::Certified,
                }],
                grounds: Vec::new(),
                status: ElectricalFactStatus::Certified,
            }],
            requirements: vec!["decouple close to package".into()],
            status: ElectricalFactStatus::Conditional,
        }),
        absolute_maximum_ratings: vec![AbsoluteMaximumRating {
            key: "supply-voltage".into(),
            value: AssertionValue::interval(Real::from(-1), Real::from(7)).unwrap(),
            units: Some("V".into()),
            conditions: Vec::new(),
            status: ElectricalFactStatus::Certified,
        }],
        recommended_operating_conditions: vec![RecommendedOperatingCondition {
            key: "supply-voltage".into(),
            value: AssertionValue::interval(Real::from(3), Real::from(5)).unwrap(),
            units: Some("V".into()),
            conditions: Vec::new(),
            status: ElectricalFactStatus::Certified,
        }],
        status: ElectricalFactStatus::Unknown,
        evidence: PartQueryEvidence::from_fact(source(), "datasheet has pinout but no die nets"),
        unknowns: vec!["internal die routing unavailable".into()],
    };

    assert!(!report.has_internal_detail());
    assert_eq!(
        VoltageRange::new(Real::from(5), Real::from(3)),
        Err(hyperparts::PartsError::InvalidVoltageEnvelope)
    );
    assert_eq!(report.pin_maps[0].internal, AssertionValue::Unknown);
}

#[test]
fn safe_connection_report_preserves_unknown_reason() {
    let report = SafeConnectionReport {
        left: terminal("shield"),
        right: terminal("chassis"),
        decision: ConnectionDecision::Unknown,
        evidence: PartQueryEvidence::from_fact(source(), "connector shell source incomplete"),
        unknowns: vec!["datasheet omits chassis bonding rule".into()],
    };

    assert_eq!(report.decision, ConnectionDecision::Unknown);
    assert_eq!(report.unknowns.len(), 1);
}

#[test]
fn typed_capabilities_are_queryable_from_tool_and_target() {
    let mut graph = PartGraph::default();
    let capability = Capability {
        tool: ToolPart {
            handle: "tool:drill-press".into(),
            part: Some(part("tool:drill-press")),
            name: "Drill press".into(),
        },
        process: Process {
            handle: "process:drill-grid-hole".into(),
            kind: ProcessKind::Subtractive,
            name: "Drill grid hole".into(),
        },
        inputs: vec![CapabilityInput {
            role: "stock".into(),
            handle: "gridbeam:blank".into(),
            quantity: AssertionValue::exact_scalar(Real::from(1)),
        }],
        outputs: vec![CapabilityOutput {
            role: "feature".into(),
            handle: "gridbeam:hole".into(),
            quantity: AssertionValue::exact_scalar(Real::from(1)),
        }],
        envelope: CapabilityEnvelope {
            materials: vec!["wood".into(), "aluminum".into()],
            conditions: vec![AssertionCondition {
                label: "spindle-speed".into(),
                value: AssertionValue::interval(Real::from(500), Real::from(2500)).unwrap(),
            }],
            safety_limits: vec![AssertionCondition {
                label: "clamp-required".into(),
                value: AssertionValue::Text("true".into()),
            }],
            status: CapabilityStatus::Conditional,
        },
        tolerances: vec![ToleranceEnvelope {
            target: "gridbeam:hole-spacing".into(),
            tolerance: AssertionValue::interval(Real::from(0), Real::from(1)).unwrap(),
            units: Some("mm".into()),
            conditions: Vec::new(),
            status: CapabilityStatus::Certified,
        }],
        fixtures: vec![FixtureRequirement {
            fixture: "fixture:grid-jig".into(),
            setup: AssertionValue::Text("clamped".into()),
            status: CapabilityStatus::Certified,
        }],
        consumables: vec![ConsumableRequirement {
            consumable: "bit:6mm".into(),
            quantity: AssertionValue::exact_scalar(Real::from(1)),
            units: Some("count".into()),
            status: CapabilityStatus::Certified,
        }],
        calibration: Some(CalibrationState {
            label: "runout".into(),
            value: AssertionValue::interval(Real::from(0), Real::from(1)).unwrap(),
            status: CapabilityStatus::NeedsReview,
        }),
        evidence: PartQueryEvidence::from_fact(source(), "grid jig fixture claim"),
        status: CapabilityStatus::Conditional,
    };
    graph.add_capability(capability.clone());
    graph.add_manufacturing_route(ManufacturingRoute {
        handle: "route:gridbeam-hole".into(),
        target: "gridbeam:hole".into(),
        operations: vec![Operation {
            handle: "op:drill".into(),
            process: capability.process.handle.clone(),
            target: "gridbeam:hole".into(),
        }],
        capabilities: vec![capability],
        unknowns: vec!["operator feed-rate omitted".into()],
        evidence: PartQueryEvidence::from_fact(source(), "manual drill route"),
        status: CapabilityStatus::NeedsReview,
    });

    assert_eq!(graph.capabilities_for_tool("tool:drill-press").len(), 1);
    assert_eq!(graph.capabilities_for_target("gridbeam:hole").len(), 1);
    assert_eq!(graph.manufacturing_routes()[0].unknowns.len(), 1);
    assert_eq!(
        graph.capabilities()[0].fixtures[0].status,
        CapabilityStatus::Certified
    );
}

#[test]
fn part_queries_return_ranked_candidates_and_unknowns() {
    let mut graph = PartGraph::default();
    let board = part("part:bench-board");
    let mut family = PartFamily::new(board.clone(), "Bench Board");
    let mut rev_a = hyperparts::PartVariant::new(variant("A"), None);
    rev_a.add_interface(Interface::new(
        "i2c",
        InterfaceKind::Electrical,
        vec![terminal("sda"), terminal("scl")],
    ));
    rev_a.add_assertion(PartAssertion::ShapeHandle(Assertion::known(
        "hypercurve:board-outline".to_string(),
        source(),
    )));
    family.insert_variant(rev_a);
    graph.insert_family(family);

    let result = graph.query_parts(&PartQuery {
        constraints: vec![
            PartConstraint::FamilyNameContains("Bench".into()),
            PartConstraint::HasInterface,
            PartConstraint::HasGeometry,
        ],
    });
    assert_eq!(result.candidates.len(), 1);
    assert_eq!(result.candidates[0].value, board);
    assert!(result.candidates[0].rank > 0);
    assert!(result.unknowns.is_empty());

    let custom = graph.query_parts(&PartQuery {
        constraints: vec![PartConstraint::Custom("adapter-only".into())],
    });
    assert_eq!(custom.candidates.len(), 0);
    assert_eq!(custom.unknowns[0].field, "adapter-only");
}

#[test]
fn eda_authoring_intake_splits_exact_part_circuit_route_and_drc_handoffs() {
    let board = part("eda:bench-board");
    let rev = variant("A");
    let result = import_eda_authoring_bundle(EdaAuthoringBundle {
        source: source(),
        part: board.clone(),
        variant: rev.clone(),
        display_name: "Bench Board".into(),
        circuit_records: vec![CircuitJsonSourceRecord {
            id: "R1".into(),
            kind: "resistor".into(),
            reference: Some("R1".into()),
            nets: vec!["VCC".into(), "OUT".into()],
            exact_fields: vec![EdaExactField {
                field: "resistance".into(),
                value: Some("10000".into()),
                unit: Some("ohm".into()),
            }],
        }],
        footprint: Some(EdaFootprintString {
            handle: "footprint:soic8".into(),
            expression: "soic:pins=8,pitch=1.27mm,width=3.9mm,leadform=gullwing".into(),
        }),
        model_references: vec![GeneratedModelReference {
            handle: "model:soic8-step".into(),
            owner: "hypercad".into(),
            format: "step".into(),
            uri: Some("pkg://bench/soic8.step".into()),
            units: Some("mm".into()),
            status: EdaModelStatus::Exact,
        }],
        package: Some(EdaPackageMetadata {
            name: "SOIC-8".into(),
            handle: "package:soic8".into(),
            terminal_count: Some(2),
            pins: vec![
                EdaPackagePin {
                    terminal: "vcc".into(),
                    name: "VCC".into(),
                    function: PinFunction::Power,
                    voltage_min: Some("3".into()),
                    voltage_max: Some("5".into()),
                },
                EdaPackagePin {
                    terminal: "gnd".into(),
                    name: "GND".into(),
                    function: PinFunction::Ground,
                    voltage_min: Some("0".into()),
                    voltage_max: Some("0".into()),
                },
            ],
        }),
        routes: vec![AutorouterOutputRecord {
            route_id: "route:vcc".into(),
            net: "VCC".into(),
            geometry_handle: Some("trace:vcc/exact".into()),
            units: Some("mm".into()),
            exact_grid: Some("0.05".into()),
            status: EdaRouteStatus::Exact,
        }],
        fabrication: vec![FabricationOutputRecord {
            artifact_id: "fab:gerbers".into(),
            format: "gerber-x2".into(),
            process: ProcessKind::Pcb,
            readiness: EdaFabricationReadiness::Ready,
            notes: vec!["fixture generated release files".into()],
        }],
    });

    assert_eq!(result.status, EdaIntakeStatus::Accepted);
    assert!(result.is_exact_ready());
    assert_eq!(result.import_report.unknown_field_count, 0);
    assert!(result.import_report.rejected_fields.is_empty());
    assert_eq!(result.circuit_handoffs[0].owner, "hypercircuit");
    assert_eq!(result.circuit_handoffs[0].status, EdaHandoffStatus::Exact);
    assert_eq!(
        result.circuit_handoffs[0].exact_parameters[0].value.clone(),
        Real::from(10000)
    );
    assert_eq!(result.route_handoffs[0].owner, "hyperpath");
    assert_eq!(result.route_handoffs[0].status, EdaHandoffStatus::Exact);
    assert_eq!(result.drc_handoffs[0].owner, "hyperdrc");
    assert_eq!(result.drc_handoffs[0].status, EdaHandoffStatus::Certified);
    assert_eq!(result.geometry_handoffs[0].status, GeometryStatus::Exact);

    let imported = result.graph.family(&board).unwrap().variant(&rev).unwrap();
    assert_eq!(
        imported.terminal(&terminal("vcc")).unwrap().polarity(),
        ElectricalPolarity::Power
    );
    assert!(imported.assertions().iter().any(|assertion| {
        matches!(
            assertion,
            PartAssertion::General(general)
                if general.key == "eda.circuit.R1.resistance"
                    && general.value == AssertionValue::exact_scalar(Real::from(10000))
        )
    }));
    assert!(matches!(
        result.graph.import_reports()[0].target,
        ImportTargetKind::Custom(ref target) if target == "tscircuit-authoring"
    ));
}

#[test]
fn eda_authoring_intake_reports_lossy_rejected_unknown_and_unresolved_inputs() {
    let board = part("eda:bad-board");
    let result = import_eda_authoring_bundle(EdaAuthoringBundle {
        source: source(),
        part: board.clone(),
        variant: variant("A"),
        display_name: "".into(),
        circuit_records: vec![CircuitJsonSourceRecord {
            id: "Rbad".into(),
            kind: "resistor".into(),
            reference: Some("R?".into()),
            nets: Vec::new(),
            exact_fields: vec![
                EdaExactField {
                    field: "resistance".into(),
                    value: Some("NaN".into()),
                    unit: Some("ohm".into()),
                },
                EdaExactField {
                    field: "tolerance".into(),
                    value: None,
                    unit: Some("%".into()),
                },
            ],
        }],
        footprint: Some(EdaFootprintString {
            handle: "footprint:bad-qfn".into(),
            expression: "qfn:pins=abc,pitch=1.2.3mm".into(),
        }),
        model_references: vec![GeneratedModelReference {
            handle: "model:preview-stl".into(),
            owner: "browser-preview".into(),
            format: "stl".into(),
            uri: None,
            units: Some("mm".into()),
            status: EdaModelStatus::LossyPreview,
        }],
        package: Some(EdaPackageMetadata {
            name: "QFN".into(),
            handle: "package:qfn".into(),
            terminal_count: Some(4),
            pins: vec![
                EdaPackagePin {
                    terminal: "".into(),
                    name: "bad".into(),
                    function: PinFunction::Unknown,
                    voltage_min: None,
                    voltage_max: None,
                },
                EdaPackagePin {
                    terminal: "io".into(),
                    name: "IO".into(),
                    function: PinFunction::Digital,
                    voltage_min: Some("0".into()),
                    voltage_max: None,
                },
            ],
        }),
        routes: vec![AutorouterOutputRecord {
            route_id: "route:missing".into(),
            net: "IO".into(),
            geometry_handle: None,
            units: Some("mm".into()),
            exact_grid: None,
            status: EdaRouteStatus::Missing,
        }],
        fabrication: vec![FabricationOutputRecord {
            artifact_id: "fab:failed".into(),
            format: "gerber-x2".into(),
            process: ProcessKind::Pcb,
            readiness: EdaFabricationReadiness::Failed,
            notes: Vec::new(),
        }],
    });

    assert_eq!(result.status, EdaIntakeStatus::NeedsReview);
    assert!(!result.is_exact_ready());
    assert!(result.import_report.unknown_field_count >= 2);
    assert!(!result.import_report.rejected_fields.is_empty());
    assert!(!result.import_report.lossy_conversions.is_empty());
    assert!(!result.import_report.unresolved_references.is_empty());
    assert_eq!(result.circuit_handoffs[0].status, EdaHandoffStatus::Unknown);
    assert_eq!(result.route_handoffs[0].status, EdaHandoffStatus::Unknown);
    assert_eq!(result.drc_handoffs[0].status, EdaHandoffStatus::Rejected);
    assert_eq!(
        result.geometry_handoffs[0].status,
        GeometryStatus::LossyMesh
    );

    let query = result.graph.query_parts(&PartQuery {
        constraints: vec![PartConstraint::HasGeometry],
    });
    assert!(query.candidates.is_empty());
    assert!(
        result
            .import_report
            .rejected_fields
            .iter()
            .any(|issue| issue.field == "footprint/pitch")
    );
    assert!(
        result
            .import_report
            .review_requirements
            .iter()
            .any(|issue| issue.field.contains("display_name"))
    );
}

proptest! {
    #[test]
    fn empty_ids_are_rejected(id in "\\PC*") {
        if id.is_empty() {
            prop_assert!(PartId::new(id).is_err());
        } else {
            prop_assert!(PartId::new(id).is_ok());
        }
    }

    #[test]
    fn physics_handles_reject_empty_identity_fields(owner in "\\PC*", handle in "\\PC*", property in "\\PC*") {
        let result = PhysicalPropertyHandle::new(
            owner.clone(),
            handle.clone(),
            property.clone(),
            None,
            PhysicalFactStatus::Exact,
        );
        if owner.is_empty() || handle.is_empty() || property.is_empty() {
            prop_assert_eq!(result, Err(hyperparts::PartsError::EmptyIdentifier));
        } else {
            prop_assert!(result.is_ok());
        }
    }

    #[test]
    fn eda_intake_keeps_decimal_circuit_parameters_exact(whole in 0_i32..10_000, frac in 0_u32..1_000) {
        let decimal = format!("{whole}.{frac:03}");
        let result = import_eda_authoring_bundle(EdaAuthoringBundle {
            source: source(),
            part: part("eda:prop-board"),
            variant: variant("A"),
            display_name: "Property Board".into(),
            circuit_records: vec![CircuitJsonSourceRecord {
                id: "Cprop".into(),
                kind: "capacitor".into(),
                reference: Some("Cprop".into()),
                nets: vec!["A".into(), "B".into()],
                exact_fields: vec![EdaExactField {
                    field: "capacitance".into(),
                    value: Some(decimal.clone()),
                    unit: Some("nF".into()),
                }],
            }],
            footprint: None,
            model_references: Vec::new(),
            package: None,
            routes: Vec::new(),
            fabrication: Vec::new(),
        });

        prop_assert_eq!(result.status, EdaIntakeStatus::Accepted);
        prop_assert_eq!(
            result.circuit_handoffs[0].exact_parameters[0].value.clone(),
            decimal.parse::<Real>().unwrap()
        );
        prop_assert!(result.import_report.rejected_fields.is_empty());
        prop_assert!(result.import_report.lossy_conversions.is_empty());
    }
}
