use std::hint::black_box;
use std::time::Instant;

use hyperparts::{
    AssertionValue, Capability, CapabilityEnvelope, CapabilityInput, CapabilityOutput,
    CapabilityStatus, ConnectionDecision, ElectricalCompatibilityReport, ElectricalFactStatus,
    ElectricalPolarity, ElectronicPackage, MaterialRequirement, PartConstraint, PartFamily,
    PartGraph, PartId, PartQuery, PartQueryEvidence, PhysicalFactStatus, PhysicalPropertyHandle,
    PhysicsHandoffReport, PinFunction, Pinout, Process, ProcessKind, Real, Terminal, TerminalId,
    ToolPart, VariantId, VoltageEnvelope, VoltageRange,
};

fn id(value: &str) -> PartId {
    PartId::new(value).unwrap()
}

fn variant(value: &str) -> VariantId {
    VariantId::new(value).unwrap()
}

fn terminal(value: &str) -> TerminalId {
    TerminalId::new(value).unwrap()
}

fn main() {
    let part = id("bench-board");
    let rev = variant("A");
    let mut family = PartFamily::new(part.clone(), "Bench Board");
    let mut part_variant = hyperparts::PartVariant::new(rev.clone(), None);
    part_variant.add_terminal(Terminal::new(
        terminal("a"),
        "A",
        ElectricalPolarity::Power,
        Some(VoltageEnvelope::new(Real::from(5), Real::from(5)).unwrap()),
    ));
    part_variant.add_terminal(Terminal::new(
        terminal("b"),
        "B",
        ElectricalPolarity::Power,
        Some(VoltageEnvelope::new(Real::from(5), Real::from(5)).unwrap()),
    ));
    family.insert_variant(part_variant);
    let mut graph = PartGraph::default();
    graph.insert_family(family);

    let iterations = 100_000_u32;
    let started = Instant::now();
    let mut safe = 0_usize;
    for _ in 0..iterations {
        let result = black_box(&graph)
            .safe_connection((&part, &rev, &terminal("a")), (&part, &rev, &terminal("b")))
            .unwrap();
        if result.value == ConnectionDecision::Safe {
            safe += 1;
        }
    }
    let elapsed = started.elapsed();
    println!(
        "safe_connection_query: {iterations} iterations in {elapsed:?} ({:?}/iter), safe={safe}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut interval_checksum = 0_usize;
    for _ in 0..iterations {
        let value = AssertionValue::interval(Real::from(1), Real::from(10)).unwrap();
        interval_checksum ^= format!("{:?}", value).len();
    }
    let elapsed = started.elapsed();
    println!(
        "assertion_interval_validation: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={interval_checksum}",
        elapsed / iterations
    );

    let source = hyperparts::SourceRef::new("bench", "physics").unwrap();
    let started = Instant::now();
    let mut physics_checksum = 0_usize;
    for _ in 0..iterations {
        let handle = PhysicalPropertyHandle::new(
            "hyperphysics",
            "material:bench/density",
            "density",
            Some("kg/m^3".into()),
            PhysicalFactStatus::Certified,
        )
        .unwrap();
        let report = PhysicsHandoffReport {
            part: part.clone(),
            handles: vec![handle],
            requirements: vec![MaterialRequirement {
                key: "density".into(),
                value: AssertionValue::exact_scalar(Real::from(1000)),
                units: Some("kg/m^3".into()),
                conditions: Vec::new(),
                status: PhysicalFactStatus::Certified,
            }],
            environments: Vec::new(),
            thermal_paths: Vec::new(),
            mechanical_load_paths: Vec::new(),
            mass_property_needs: Vec::new(),
            status: PhysicalFactStatus::Certified,
            evidence: PartQueryEvidence::from_fact(source.clone(), "bench physical handoff"),
            unknowns: Vec::new(),
        };
        physics_checksum ^= usize::from(report.is_certified_ready());
    }
    let elapsed = started.elapsed();
    println!(
        "physics_handoff_construction: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={physics_checksum}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut electrical_checksum = 0_usize;
    for _ in 0..iterations {
        let report = ElectricalCompatibilityReport {
            part: part.clone(),
            package: Some(ElectronicPackage {
                name: "SOT-23".into(),
                handle: "package:sot-23".into(),
                terminal_count: Some(3),
                status: ElectricalFactStatus::Certified,
            }),
            pinout: vec![Pinout {
                terminal: terminal("vcc"),
                name: "VCC".into(),
                function: PinFunction::Power,
                voltage: Some(VoltageRange::new(Real::from(3), Real::from(5)).unwrap()),
                current: None,
                status: ElectricalFactStatus::Certified,
            }],
            pin_maps: Vec::new(),
            die_ports: Vec::new(),
            die_nets: Vec::new(),
            power_intent: None,
            absolute_maximum_ratings: Vec::new(),
            recommended_operating_conditions: Vec::new(),
            status: ElectricalFactStatus::Unknown,
            evidence: PartQueryEvidence::from_fact(source.clone(), "bench electrical handoff"),
            unknowns: vec!["internal die detail unavailable".into()],
        };
        electrical_checksum ^= usize::from(report.has_internal_detail());
    }
    let elapsed = started.elapsed();
    println!(
        "electrical_handoff_construction: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={electrical_checksum}",
        elapsed / iterations
    );

    let mut capability_graph = PartGraph::default();
    capability_graph.add_capability(Capability {
        tool: ToolPart {
            handle: "tool:bench-router".into(),
            part: None,
            name: "Bench router".into(),
        },
        process: Process {
            handle: "process:profile-cut".into(),
            kind: ProcessKind::Subtractive,
            name: "Profile cut".into(),
        },
        inputs: vec![CapabilityInput {
            role: "stock".into(),
            handle: "stock:panel".into(),
            quantity: AssertionValue::exact_scalar(Real::from(1)),
        }],
        outputs: vec![CapabilityOutput {
            role: "part".into(),
            handle: "part:panel".into(),
            quantity: AssertionValue::exact_scalar(Real::from(1)),
        }],
        envelope: CapabilityEnvelope {
            materials: vec!["plywood".into()],
            conditions: Vec::new(),
            safety_limits: Vec::new(),
            status: CapabilityStatus::NeedsReview,
        },
        tolerances: Vec::new(),
        fixtures: Vec::new(),
        consumables: Vec::new(),
        calibration: None,
        evidence: PartQueryEvidence::from_fact(source, "bench capability"),
        status: CapabilityStatus::NeedsReview,
    });
    let started = Instant::now();
    let mut capability_checksum = 0_usize;
    for _ in 0..iterations {
        capability_checksum ^= black_box(&capability_graph)
            .capabilities_for_target("part:panel")
            .len();
    }
    let elapsed = started.elapsed();
    println!(
        "capability_target_query: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={capability_checksum}",
        elapsed / iterations
    );

    let part_query = PartQuery {
        constraints: vec![PartConstraint::FamilyNameContains("Bench".into())],
    };
    let started = Instant::now();
    let mut query_checksum = 0_usize;
    for _ in 0..iterations {
        query_checksum ^= black_box(&graph)
            .query_parts(black_box(&part_query))
            .candidates
            .len();
    }
    let elapsed = started.elapsed();
    println!(
        "part_query: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={query_checksum}",
        elapsed / iterations
    );
}
