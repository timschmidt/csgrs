#![cfg(feature = "drc")]

use hypercircuit::{
    BoardOutline, CheckedProject, Design, DesignModule, DeviceModelKind, DrcHandoffOmission,
    Footprint, LandPatternBody, LandPatternPad, LayoutTransform, MaterializationOptions, NetClass,
    NetClassId, PCB_LOSS_TANGENT_PROPERTY, PCB_RELATIVE_PERMITTIVITY_PROPERTY, PadId, PadShape,
    Part, PcbMaterialPropertyIssue, PcbMaterialPropertyLibrary, PlacementRule, Plating,
    PortDirection, Real, ReleaseBlocker, ReleasePreparationOptions, Route, Via, ViaMaskIntent,
    parts, pin,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;
use hyperphysics::{
    MaterialAssertion, MaterialPropertyGraph, MaterialPropertyKind, MaterialState, PropertyValue,
    SourceSpec,
};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn body() -> LandPatternBody {
    LandPatternBody {
        outline: vec![point(-3, -4), point(3, -4), point(3, 4), point(-3, 4)],
        height: Real::from(3),
        standoff: (Real::one() / Real::from(5)).unwrap(),
    }
}

fn fr4_properties() -> MaterialPropertyGraph {
    let mut graph = MaterialPropertyGraph::default();
    for (property, value) in [
        (
            PCB_RELATIVE_PERMITTIVITY_PROPERTY,
            (Real::from(42) / Real::from(10)).unwrap(),
        ),
        (
            PCB_LOSS_TANGENT_PROPERTY,
            (Real::from(18) / Real::from(1_000)).unwrap(),
        ),
    ] {
        graph.push(MaterialAssertion {
            kind: MaterialPropertyKind::Custom(property.into()),
            value: PropertyValue::exact_scalar(value),
            unit: "1".into(),
            state: MaterialState::Cured,
            condition: Some("datasheet nominal test condition".into()),
            source: SourceSpec::new("laminate-datasheet", "revision-a"),
        });
    }
    graph
}

fn two_pad_body() -> Footprint {
    let pad = |id: &str, y: i64| LandPatternPad {
        id: PadId::new(id).unwrap(),
        center: point(0, y),
        rotation_degrees: Real::zero(),
        copper_layers: vec![TraceLayer(0)],
        shape: PadShape::Rectangle {
            width: Real::from(2),
            height: Real::from(2),
        },
        drill: None,
        plating: Plating::Unspecified,
        solder_mask_margin: None,
        paste_margin: None,
    };
    Footprint::new()
        .pad(pad("1", -2))
        .pad(pad("2", 2))
        .body(body())
}

fn narrow_horizontal_two_pad_body() -> Footprint {
    let pad = |id: &str, x: i64| LandPatternPad {
        id: PadId::new(id).unwrap(),
        center: point(x, 0),
        rotation_degrees: Real::zero(),
        copper_layers: vec![TraceLayer(0)],
        shape: PadShape::Rectangle {
            width: Real::one(),
            height: Real::from(2),
        },
        drill: None,
        plating: Plating::Unspecified,
        solder_mask_margin: None,
        paste_margin: None,
    };
    Footprint::new()
        .pad(pad("1", -1))
        .pad(pad("2", 1))
        .body(body())
}

fn fluent_release_design() -> hypercircuit::CheckedDesign {
    let mut design = Design::new(
        "fluent-release",
        BoardOutline::rectangle(Real::from(30), Real::from(20)),
        hypercircuit::PcbStackup::two_layer(
            (Real::from(35) / Real::from(1_000)).unwrap(),
            (Real::from(153) / Real::from(100)).unwrap(),
            Some("hyperphysics:copper".into()),
            Some("hyperphysics:FR4".into()),
        ),
    )
    .unwrap();
    let supply = design.signal("VCC").unwrap();
    let ground = design.ground("GND").unwrap();
    let source = design
        .add(
            Part::new("V1", "voltage source")
                .model_kind(DeviceModelKind::VoltageSource)
                .part_ref("parts:voltage-source")
                .pin(pin("pos").power_output().pad("1"))
                .pin(pin("neg").power_output().pad("2"))
                .parameter("voltage", Real::from(5), "V")
                .footprint(two_pad_body())
                .at(point(5, 5)),
        )
        .unwrap();
    let load = design
        .add(
            parts::resistor("R1", Real::from(1_000))
                .part_ref("parts:resistor")
                .footprint(two_pad_body())
                .at(point(25, 5)),
        )
        .unwrap();
    design
        .connect(
            &supply,
            [source.pin("pos").unwrap(), load.pin("1").unwrap()],
        )
        .unwrap();
    design
        .connect(
            &ground,
            [source.pin("neg").unwrap(), load.pin("2").unwrap()],
        )
        .unwrap();
    design
        .constrain(PlacementRule::fixed("source-origin", &source, point(5, 5)))
        .unwrap();
    design
        .constrain(PlacementRule::relative(
            "load-after-source",
            &load,
            &source,
            point(20, 0),
        ))
        .unwrap();
    design
        .route(
            &supply,
            Route::new("supply-route", TraceLayer(0), Real::one()).line(point(5, 3), point(25, 3)),
        )
        .unwrap();
    design
        .route(
            &ground,
            Route::new("ground-route", TraceLayer(0), Real::one())
                .line(point(5, 7), point(5, 10))
                .line(point(5, 10), point(25, 10))
                .line(point(25, 10), point(25, 7)),
        )
        .unwrap();
    design
        .via(
            &supply,
            Via::new(
                "supply-via",
                TraceLayer(0),
                TraceLayer(1),
                point(15, 3),
                Real::from(3),
                Real::one(),
            )
            .mask(ViaMaskIntent::tented()),
        )
        .unwrap();
    design.finish().unwrap()
}

fn offset_uncertain_release_design() -> hypercircuit::CheckedDesign {
    let mut design = Design::new(
        "offset-uncertain-release",
        BoardOutline::rectangle(Real::from(30), Real::from(20)),
        hypercircuit::PcbStackup::two_layer(
            (Real::from(35) / Real::from(1_000)).unwrap(),
            (Real::from(153) / Real::from(100)).unwrap(),
            Some("hyperphysics:copper".into()),
            Some("hyperphysics:FR4".into()),
        ),
    )
    .unwrap();
    let supply = design.signal("VCC").unwrap();
    let ground = design.ground("GND").unwrap();
    let source = design
        .add(
            Part::new("V1", "voltage source")
                .model_kind(DeviceModelKind::VoltageSource)
                .part_ref("parts:voltage-source")
                .pin(pin("pos").power_output().pad("1"))
                .pin(pin("neg").power_output().pad("2"))
                .parameter("voltage", Real::from(5), "V")
                .footprint(narrow_horizontal_two_pad_body())
                .at(point(5, 6)),
        )
        .unwrap();
    let load = design
        .add(
            parts::resistor("R1", Real::from(1_000))
                .part_ref("parts:resistor")
                .footprint(narrow_horizontal_two_pad_body())
                .at(point(22, 6)),
        )
        .unwrap();
    design
        .connect(
            &supply,
            [source.pin("pos").unwrap(), load.pin("1").unwrap()],
        )
        .unwrap();
    design
        .connect(
            &ground,
            [source.pin("neg").unwrap(), load.pin("2").unwrap()],
        )
        .unwrap();
    design
        .route(
            &supply,
            Route::new("supply-route", TraceLayer(0), Real::one())
                .line(point(4, 6), point(4, 3))
                .line(point(4, 3), point(21, 3))
                .line(point(21, 3), point(21, 6)),
        )
        .unwrap();
    design
        .route(
            &ground,
            Route::new("ground-route", TraceLayer(0), Real::one())
                .line(point(6, 6), point(6, 10))
                .line(point(6, 10), point(23, 10))
                .line(point(23, 10), point(23, 6)),
        )
        .unwrap();
    design
        .via(
            &supply,
            Via::new(
                "supply-via",
                TraceLayer(0),
                TraceLayer(1),
                point(12, 3),
                Real::from(2),
                Real::one(),
            )
            .mask(ViaMaskIntent::tented()),
        )
        .unwrap();
    design.finish().unwrap()
}

fn hierarchical_release_project() -> CheckedProject {
    let stackup = || {
        hypercircuit::PcbStackup::two_layer(
            (Real::from(35) / Real::from(1_000)).unwrap(),
            (Real::from(153) / Real::from(100)).unwrap(),
            Some("hyperphysics:copper".into()),
            Some("hyperphysics:FR4".into()),
        )
    };
    let mut load_design = Design::new(
        "load-circuit",
        BoardOutline::rectangle(Real::from(30), Real::from(20)),
        stackup(),
    )
    .unwrap();
    let load_input = load_design.signal("IN").unwrap();
    let load_ground = load_design.ground("GND").unwrap();
    let load_input_port = load_design
        .port("input", &load_input, PortDirection::Input, false)
        .unwrap();
    let load_ground_port = load_design
        .port("ground", &load_ground, PortDirection::Ground, false)
        .unwrap();
    let load = load_design
        .add(
            parts::resistor("R1", Real::from(1_000))
                .part_ref("parts:resistor")
                .footprint(two_pad_body())
                .at(point(5, 5)),
        )
        .unwrap();
    load_design
        .connect(&load_input, [load.pin("1").unwrap()])
        .unwrap();
    load_design
        .connect(&load_ground, [load.pin("2").unwrap()])
        .unwrap();
    load_design
        .constrain(PlacementRule::fixed("load-origin", &load, point(5, 5)))
        .unwrap();
    let load_module = DesignModule::new("load-layout", load_design.finish().unwrap()).unwrap();

    let mut root_design = Design::new(
        "hierarchical-release",
        BoardOutline::rectangle(Real::from(30), Real::from(20)),
        stackup(),
    )
    .unwrap();
    let supply = root_design.signal("VCC").unwrap();
    let ground = root_design.ground("GND").unwrap();
    let source = root_design
        .add(
            Part::new("V1", "voltage source")
                .model_kind(DeviceModelKind::VoltageSource)
                .part_ref("parts:voltage-source")
                .pin(pin("pos").power_output().pad("1"))
                .pin(pin("neg").power_output().pad("2"))
                .parameter("voltage", Real::from(5), "V")
                .footprint(two_pad_body())
                .at(point(5, 5)),
        )
        .unwrap();
    root_design
        .connect(&supply, [source.pin("pos").unwrap()])
        .unwrap();
    root_design
        .connect(&ground, [source.pin("neg").unwrap()])
        .unwrap();
    root_design
        .constrain(PlacementRule::fixed("source-origin", &source, point(5, 5)))
        .unwrap();
    let mut root =
        DesignModule::new("hierarchical-release-layout", root_design.finish().unwrap()).unwrap();
    root.instantiate(
        "load",
        load_module,
        [(&load_input_port, &supply), (&load_ground_port, &ground)],
        LayoutTransform {
            position: point(15, 0),
            ..LayoutTransform::default()
        },
    )
    .unwrap();
    root.compile().unwrap()
}

#[test]
fn checked_fluent_design_prepares_cohesive_release_evidence() {
    let checked = fluent_release_design();
    let dc = checked
        .circuit
        .linear_mna_from_devices()
        .unwrap()
        .solve_exact()
        .unwrap();
    assert!(dc.replay.accepted);
    assert_eq!(dc.candidate[0], Real::from(5));

    let report = checked
        .prepare_release(ReleasePreparationOptions {
            materialization: MaterializationOptions::default(),
            ..ReleasePreparationOptions::default()
        })
        .unwrap();

    assert!(report.placement.is_satisfied());
    assert_eq!(report.resolved_layout.placements[1].position, point(25, 5));
    assert!(report.erc.is_valid());
    assert!(report.drc.is_release_clean());
    assert!(report.fabrication_integrity.is_empty());
    assert!(report.cam_round_trip.is_release_clean());
    assert!(report.assembly_round_trip.is_release_clean());
    assert_eq!(report.assembly.pick_and_place.len(), 2);
    assert!(report.is_release_clean(), "{:?}", report.release_blockers());
}

#[test]
fn release_rule_failures_remain_inspectable_reports() {
    let mut checked = fluent_release_design();
    checked.layout.land_patterns[0].body = None;

    let report = checked
        .prepare_release(ReleasePreparationOptions::default())
        .unwrap();

    assert!(report.drc_handoff.omissions.iter().any(|omission| {
        matches!(omission, DrcHandoffOmission::MissingComponentEnvelope(instance) if instance == "V1")
    }));
    assert!(!report.is_release_clean());
    assert!(report.release_blockers().iter().any(
        |blocker| matches!(blocker, ReleaseBlocker::DrcHandoffOmissions(count) if *count == 1)
    ));
    assert!(
        report
            .release_blockers()
            .iter()
            .any(|blocker| matches!(blocker, ReleaseBlocker::DrcErrors(count) if *count >= 1))
    );
}

#[test]
fn release_resolves_controlled_impedance_materials_through_hyperphysics() {
    let mut checked = fluent_release_design();
    checked.layout.rules.net_classes.push(NetClass {
        id: NetClassId::new("controlled-supply").unwrap(),
        parent: None,
        nets: vec![hypercircuit::NetId::new("VCC").unwrap()],
        min_trace_width: None,
        preferred_trace_width: None,
        min_clearance: None,
        preferred_via_land_diameter: None,
        preferred_via_drill_diameter: None,
        preferred_via_style: None,
        max_length: None,
        max_via_count: None,
        target_impedance_ohms: Some(Real::from(50)),
        impedance_tolerance_ohms: Some(Real::from(5)),
        requires_reference_plane: true,
    });

    let unresolved = checked
        .prepare_release(ReleasePreparationOptions::default())
        .unwrap();
    assert_eq!(
        unresolved
            .drc_handoff
            .omissions
            .iter()
            .filter(|omission| matches!(
                omission,
                DrcHandoffOmission::UnresolvedDielectricProperty {
                    issue: PcbMaterialPropertyIssue::Unknown,
                    ..
                }
            ))
            .count(),
        2
    );
    assert!(unresolved.release_blockers().iter().any(|blocker| matches!(
        blocker,
        ReleaseBlocker::DrcHandoffOmissions(count) if *count == 2
    )));

    let resolved = checked
        .prepare_release(ReleasePreparationOptions {
            pcb_materials: PcbMaterialPropertyLibrary::default()
                .with_material("hyperphysics:FR4", fr4_properties()),
            ..ReleasePreparationOptions::default()
        })
        .unwrap();
    assert!(resolved.drc_handoff.omissions.is_empty());
    assert_eq!(
        resolved.drc_handoff.stackup.material_dielectric_constant,
        Some((Real::from(42) / Real::from(10)).unwrap())
    );
    assert_eq!(resolved.drc_handoff.dielectric_material_evidence.len(), 1);
    assert_eq!(
        resolved.drc_handoff.dielectric_material_evidence[0].relative_permittivity_sources[0]
            .authority,
        "laminate-datasheet"
    );
    assert!(resolved.drc.violations.iter().any(|violation| {
        violation.check == "net-impedance-target-readiness"
            && violation.severity == hyperdrc::Severity::Error
            && violation.message.as_deref().is_some_and(|message| {
                message.contains("estimated outer microstrip impedance")
                    && message.contains("outside target")
            })
    }));
    assert!(
        !resolved
            .release_blockers()
            .iter()
            .any(|blocker| matches!(blocker, ReleaseBlocker::DrcHandoffOmissions(_)))
    );
    assert!(
        resolved
            .release_blockers()
            .iter()
            .any(|blocker| matches!(blocker, ReleaseBlocker::DrcErrors(count) if *count >= 1))
    );
}

#[test]
fn centered_pad_materialization_avoids_spurious_offset_uncertainty() {
    let report = offset_uncertain_release_design()
        .prepare_release(ReleasePreparationOptions::default())
        .unwrap();

    assert!(
        !report
            .drc
            .violations
            .iter()
            .any(|violation| violation.check == "geometry-uncertainty")
    );
    assert!(report.is_release_clean(), "{:?}", report.release_blockers());
}

#[test]
fn checked_hierarchy_prepares_release_from_its_path_qualified_flat_view() {
    let project = hierarchical_release_project();
    let simulation = project
        .composed
        .circuit
        .linear_mna_from_devices()
        .unwrap()
        .solve_exact()
        .unwrap();
    assert!(simulation.replay.accepted);

    let report = project
        .prepare_release(ReleasePreparationOptions::default())
        .unwrap();

    assert!(
        report
            .resolved_layout
            .placements
            .iter()
            .any(|placement| placement.instance.as_str() == "load/R1"
                && placement.position == point(20, 5))
    );
    assert!(
        report
            .materialization
            .copper_features
            .iter()
            .any(|feature| feature.source.contains("load/R1"))
    );
    assert!(report.is_release_clean(), "{:?}", report.release_blockers());
    assert_eq!(project.sources.len(), 2);
    assert_eq!(project.schematics.len(), 2);
}
