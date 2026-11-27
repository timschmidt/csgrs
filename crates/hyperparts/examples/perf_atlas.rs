//! Performance atlas for HyperParts: part-graph construction and queries,
//! exact electrical-fact decisions, and EDA authoring intake.
//!
//! Axes:
//! - graph size: part families (2^0..2^14), constraints per query, typed
//!   capabilities and the position of the matching capability;
//! - exact facts: voltage-bound operand bits (integer, rational and irrational
//!   bounds) and the gap between two envelopes (`2^-k`, down to touching);
//! - EDA intake: circuit records, digits per exact field value, package pins
//!   and footprint parameters, with clean, rational, malformed and omitted
//!   source values.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperparts --all-features --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
//! With `--resources`, query results report their candidate count and intake
//! results their audit-issue count as `nodes`; with `--counters` (and
//! `dispatch-trace`), Hyperreal dispatch events are recorded.
use hyperparts::{
    Assertion, AssertionValue, AutorouterOutputRecord, Capability, CapabilityEnvelope,
    CapabilityInput, CapabilityOutput, CapabilityStatus, CircuitJsonSourceRecord,
    ConnectionDecision, EdaAuthoringBundle, EdaAuthoringImportResult, EdaExactField,
    EdaFabricationReadiness, EdaFootprintString, EdaIntakeStatus, EdaModelStatus,
    EdaPackageMetadata, EdaPackagePin, EdaRouteStatus, ElectricalPolarity, FabricationOutputRecord,
    GeneratedModelReference, Interface, InterfaceKind, PartAssertion, PartConstraint, PartFamily,
    PartGraph, PartId, PartQuery, PartQueryEvidence, PartVariant, PinFunction, Process,
    ProcessKind, QueryResult, Real, SourceRef, Terminal, TerminalId, ToolPart, VariantId,
    VoltageEnvelope, import_eda_authoring_bundle,
};
use hyperreal::Rational;
use perf_atlas::{Atlas, Outcome, Rng, Shape, ladder, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

#[cfg(feature = "dispatch-trace")]
fn dispatch_events() -> u64 {
    hyperreal::dispatch_trace::snapshot()
        .iter()
        .map(|count| count.count)
        .sum()
}

fn source() -> SourceRef {
    SourceRef::new("atlas", "generated").expect("non-empty source")
}

fn part_id(index: usize) -> PartId {
    PartId::new(format!("part:{index:06}")).expect("non-empty part id")
}

fn variant_id() -> VariantId {
    VariantId::new("A").expect("non-empty variant id")
}

fn terminal_id(name: &str) -> TerminalId {
    TerminalId::new(name).expect("non-empty terminal id")
}

fn volts(min: i64, max: i64) -> Option<VoltageEnvelope> {
    VoltageEnvelope::new(Real::from(min), Real::from(max)).ok()
}

/// A family with one variant. Rich families carry terminals, an interface, a
/// geometry handle and a general assertion; minimal ones only the variant.
fn family(index: usize, rich: bool) -> PartFamily {
    let mut variant = PartVariant::new(variant_id(), None);
    if rich {
        variant.add_terminal(Terminal::new(
            terminal_id("vcc"),
            "VCC",
            ElectricalPolarity::Power,
            volts(3, 5),
        ));
        variant.add_terminal(Terminal::new(
            terminal_id("gnd"),
            "GND",
            ElectricalPolarity::Ground,
            None,
        ));
        variant.add_terminal(Terminal::new(
            terminal_id("io"),
            "IO",
            ElectricalPolarity::Signal,
            None,
        ));
        variant.add_interface(Interface::new(
            "power",
            InterfaceKind::Electrical,
            vec![terminal_id("vcc"), terminal_id("gnd")],
        ));
        // Every other family has geometry, so `HasGeometry` matches half.
        if index.is_multiple_of(2) {
            variant.add_assertion(PartAssertion::ShapeHandle(Assertion::known(
                format!("hypercad:body:{index}"),
                source(),
            )));
        }
    }
    let mut family = PartFamily::new(part_id(index), format!("Family {index:06}"));
    family.insert_variant(variant);
    family
}

fn graph(families: usize, rich: bool) -> PartGraph {
    let mut graph = PartGraph::default();
    for index in 0..families {
        graph.insert_family(family(index, rich));
    }
    graph
}

/// A routing capability whose output is `target`.
fn capability(index: usize, target: &str) -> Capability {
    let quantity = AssertionValue::exact_scalar(Real::from(1));
    Capability {
        tool: ToolPart {
            handle: format!("tool:router-{index}"),
            part: None,
            name: format!("Router {index}"),
        },
        process: Process {
            handle: format!("process:profile-{index}"),
            kind: ProcessKind::Subtractive,
            name: "Profile cut".into(),
        },
        inputs: vec![CapabilityInput {
            role: "stock".into(),
            handle: format!("stock:{index}"),
            quantity: quantity.clone(),
        }],
        outputs: vec![CapabilityOutput {
            role: "part".into(),
            handle: target.into(),
            quantity,
        }],
        envelope: CapabilityEnvelope {
            materials: vec!["aluminium".into()],
            conditions: Vec::new(),
            safety_limits: Vec::new(),
            status: CapabilityStatus::Certified,
        },
        tolerances: Vec::new(),
        fixtures: Vec::new(),
        consumables: Vec::new(),
        calibration: None,
        evidence: PartQueryEvidence::from_fact(source(), "atlas capability"),
        status: CapabilityStatus::Certified,
    }
}

const TARGET: &str = "part:bracket";

/// `count` capabilities. The match is every capability ("all"), only the last
/// one ("last"), or none ("miss").
fn capabilities(count: usize, kind: &str) -> PartGraph {
    let mut graph = PartGraph::default();
    for index in 0..count {
        let hit = match kind {
            "all" => true,
            "last" => index + 1 == count,
            _ => false,
        };
        let target = if hit {
            TARGET.to_owned()
        } else {
            format!("part:other-{index}")
        };
        graph.add_capability(capability(index, &target));
    }
    graph
}

/// A query result; every ranked answer, including no candidates, succeeded.
/// Its candidate count is reported as `nodes`.
struct Found<T>(QueryResult<T>);
impl<T> Outcome for Found<T> {
    fn succeeded(&self) -> bool {
        true
    }
    fn shape(&self) -> Option<Shape> {
        Some(Shape {
            nodes: self.0.candidates.len() as u64,
            depth: 0,
            bits: 0,
        })
    }
}

fn candidates<T>(result: QueryResult<T>) -> Found<T> {
    Found(result)
}

/// A built graph; its family count is reported as `nodes`.
struct Built(PartGraph);
impl Outcome for Built {
    fn succeeded(&self) -> bool {
        true
    }
    fn shape(&self) -> Option<Shape> {
        let all = self.0.query_parts(&PartQuery {
            constraints: Vec::new(),
        });
        Some(Shape {
            nodes: all.candidates.len() as u64,
            depth: 0,
            bits: 0,
        })
    }
}

/// `2^k` as an exact rational, for any `k >= 0`.
fn pow2(k: u32) -> Rational {
    let mut out = Rational::new(1);
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        out = &out * &Rational::new(1 << step);
        left -= step;
    }
    out
}

/// A positive value of the given kind whose parts have about `bits` bits.
fn bound(kind: &str, bits: u32, rng: &mut Rng) -> Real {
    let integer = |rng: &mut Rng| -> Rational { rng.decimal(bits).parse().expect("digits") };
    match kind {
        "integer" => Real::new(integer(rng)),
        "rational" => Real::new(&integer(rng) / &integer(rng)),
        // Odd radicands are, with high probability, not perfect squares.
        _ => {
            let radicand = &(&integer(rng) * &Rational::new(2)) + &Rational::new(1);
            Real::new(radicand).sqrt().expect("positive radicand")
        }
    }
}

/// Two power terminals in one graph, with the given voltage envelopes.
fn connection_graph(left: VoltageEnvelope, right: VoltageEnvelope) -> PartGraph {
    let mut graph = PartGraph::default();
    for (index, envelope) in [left, right].into_iter().enumerate() {
        let mut variant = PartVariant::new(variant_id(), None);
        variant.add_terminal(Terminal::new(
            terminal_id("vcc"),
            "VCC",
            ElectricalPolarity::Power,
            Some(envelope),
        ));
        let mut family = PartFamily::new(part_id(index), "rail");
        family.insert_variant(variant);
        graph.insert_family(family);
    }
    graph
}

/// The connection decision, with `Unknown` marked as a failure.
fn decide(graph: &PartGraph, left: usize, right: usize) -> Option<ConnectionDecision> {
    let (variant, terminal) = (variant_id(), terminal_id("vcc"));
    let decision = graph
        .safe_connection(
            (&part_id(left), &variant, &terminal),
            (&part_id(right), &variant, &terminal),
        )
        .ok()?
        .value;
    (decision != ConnectionDecision::Unknown).then_some(decision)
}

/// One exact source field value of the given kind.
fn field_value(kind: &str, digits: usize, rng: &mut Rng) -> Option<String> {
    let number = |rng: &mut Rng| -> String {
        (0..digits.max(1))
            .map(|i| {
                let low = u64::from(i == 0);
                char::from(b'0' + (low + rng.below(10 - low)) as u8)
            })
            .collect()
    };
    match kind {
        "decimal" => {
            let whole = number(rng);
            Some(format!("{whole}.{}", number(rng)))
        }
        "rational" => {
            let (p, q) = (number(rng), number(rng));
            Some(format!("{p}/{q}"))
        }
        "malformed" => Some(format!("{}_{}", number(rng), number(rng))),
        _ => None,
    }
}

const FIELD_KINDS: [&str; 4] = ["decimal", "rational", "malformed", "omitted"];

/// A tscircuit-style bundle. `records` circuit records each carry two exact
/// fields; `pins` package pins carry exact voltage bounds; the footprint
/// carries `parameters` numeric parameters; `routes` routes and fabrication
/// outputs scale with `routes`.
struct BundleShape {
    records: usize,
    digits: usize,
    pins: usize,
    parameters: usize,
    routes: usize,
}

fn bundle(kind: &str, shape: &BundleShape, rng: &mut Rng) -> EdaAuthoringBundle {
    let circuit_records = (0..shape.records)
        .map(|index| CircuitJsonSourceRecord {
            id: format!("R{index}"),
            kind: "resistor".into(),
            reference: Some(format!("R{index}")),
            nets: vec![format!("N{index}"), format!("N{}", index + 1)],
            exact_fields: vec![
                EdaExactField {
                    field: "resistance".into(),
                    value: field_value(kind, shape.digits, rng),
                    unit: Some("ohm".into()),
                },
                EdaExactField {
                    field: "tolerance".into(),
                    value: field_value(kind, shape.digits, rng),
                    unit: Some("%".into()),
                },
            ],
        })
        .collect();
    let parameters: Vec<String> = (0..shape.parameters)
        .map(|index| format!("p{index}={}mm", 1 + index % 7))
        .collect();
    EdaAuthoringBundle {
        source: source(),
        part: part_id(0),
        variant: variant_id(),
        display_name: "Atlas board".into(),
        circuit_records,
        footprint: Some(EdaFootprintString {
            handle: "footprint:generated".into(),
            expression: format!("soic:{}", parameters.join(",")),
        }),
        model_references: vec![GeneratedModelReference {
            handle: "model:body".into(),
            owner: "hypercad".into(),
            format: "step".into(),
            uri: None,
            units: Some("mm".into()),
            status: EdaModelStatus::Exact,
        }],
        package: Some(EdaPackageMetadata {
            name: "QFN".into(),
            handle: "package:qfn".into(),
            terminal_count: Some(shape.pins),
            pins: (0..shape.pins)
                .map(|index| EdaPackagePin {
                    terminal: format!("p{index}"),
                    name: format!("P{index}"),
                    function: PinFunction::Power,
                    voltage_min: Some("0".into()),
                    voltage_max: Some(format!("{}.3", 1 + index % 5)),
                })
                .collect(),
        }),
        routes: (0..shape.routes)
            .map(|index| AutorouterOutputRecord {
                route_id: format!("route:{index}"),
                net: format!("N{index}"),
                geometry_handle: Some(format!("trace:{index}")),
                units: Some("mm".into()),
                exact_grid: Some("0.05".into()),
                status: EdaRouteStatus::Exact,
            })
            .collect(),
        fabrication: (0..shape.routes)
            .map(|index| FabricationOutputRecord {
                artifact_id: format!("fab:{index}"),
                format: "gerber-x2".into(),
                process: ProcessKind::Pcb,
                readiness: EdaFabricationReadiness::Ready,
                notes: Vec::new(),
            })
            .collect(),
    }
}

/// An intake result. Only a `Rejected` intake (nothing importable) is a
/// failure; reviewed and unknown-bearing intakes are valid outcomes. The
/// number of recorded audit issues is reported as `nodes`.
struct Intake(EdaAuthoringImportResult);
impl Outcome for Intake {
    fn succeeded(&self) -> bool {
        self.0.status != EdaIntakeStatus::Rejected
    }
    fn reason(&self) -> Option<String> {
        Some(format!("{:?}", self.0.status))
    }
    fn shape(&self) -> Option<Shape> {
        let report = &self.0.import_report;
        let issues = report.parsed_assertions.len()
            + report.rejected_fields.len()
            + report.lossy_conversions.len()
            + report.stale_sources.len()
            + report.unresolved_references.len()
            + report.license_notes.len()
            + report.review_requirements.len();
        Some(Shape {
            nodes: issues as u64,
            depth: 0,
            bits: 0,
        })
    }
}

fn intake(result: EdaAuthoringImportResult) -> Intake {
    Intake(result)
}

fn main() {
    let mut atlas = Atlas::from_args("hyperparts");
    #[cfg(feature = "dispatch-trace")]
    let _recording = atlas.config().counters.then(|| {
        atlas.counter("dispatch events", dispatch_events);
        hyperreal::dispatch_trace::recording_scope()
    });
    let families = powers_of_two(0, 14);

    atlas.operation(
        "graph",
        "build part graph",
        "part families",
        "Construct and insert families (one variant each) into a new graph; rich families carry three terminals with an exact voltage envelope, an interface and, for every other family, a geometry handle.",
    );
    for (kind, rich) in [("minimal", false), ("rich", true)] {
        atlas.sweep(
            kind,
            &families,
            |n, _| n as usize,
            |&n| Built(graph(n, rich)),
        );
    }

    // Queries read the graph without caching anything on it, so one built
    // graph serves every call; building one per call would dominate the run.
    atlas.reuse_inputs(true);
    atlas.operation(
        "graph",
        "query_parts",
        "part families",
        "Rank families against one constraint over a graph of rich families: a name every family matches, a name none matches, geometry (half match), an interface (all match), or a capability target present in the graph.",
    );
    type Constraint = fn() -> PartConstraint;
    let constraints: [(&str, Constraint); 5] = [
        ("name, all match", || {
            PartConstraint::FamilyNameContains("Family".into())
        }),
        ("name, none match", || {
            PartConstraint::FamilyNameContains("absent".into())
        }),
        ("geometry, half match", || PartConstraint::HasGeometry),
        ("interface, all match", || PartConstraint::HasInterface),
        ("capability target", || {
            PartConstraint::HasCapabilityTarget(TARGET.into())
        }),
    ];
    for (kind, constraint) in constraints {
        atlas.sweep(
            kind,
            &families,
            |n, _| {
                let mut graph = graph(n as usize, true);
                graph.add_capability(capability(0, TARGET));
                (
                    graph,
                    PartQuery {
                        constraints: vec![constraint()],
                    },
                )
            },
            |(graph, query)| candidates(graph.query_parts(query)),
        );
    }
    atlas.grid(
        "name constraints, all match",
        ("part families", &powers_of_two(0, 12)),
        ("constraints per query", &powers_of_two(0, 6)),
        |n, c, _| {
            (
                graph(n as usize, true),
                PartQuery {
                    constraints: (0..c as usize)
                        .map(|_| PartConstraint::FamilyNameContains("Family".into()))
                        .collect(),
                },
            )
        },
        |(graph, query)| candidates(graph.query_parts(query)),
    );

    atlas.operation(
        "graph",
        "capabilities_for_target",
        "typed capabilities",
        "Collect the capabilities producing a target; the target is produced by every capability, only the last, or none.",
    );
    let counts = powers_of_two(0, 16);
    for kind in ["all", "last", "miss"] {
        atlas.sweep(
            kind,
            &counts,
            |n, _| capabilities(n as usize, kind),
            |graph| graph.capabilities_for_target(TARGET).len(),
        );
    }
    atlas.operation(
        "graph",
        "has_capability_target",
        "typed capabilities",
        "Short-circuiting existence test for a capability target; found at the first capability, at the last, or missing (every capability scanned).",
    );
    for kind in ["all", "last", "miss"] {
        atlas.sweep(
            kind,
            &counts,
            |n, _| capabilities(n as usize, kind),
            |graph| u8::from(graph.has_capability_target(TARGET)),
        );
    }

    atlas.operation(
        "graph",
        "safe_connection by graph size",
        "part families",
        "Decide whether two terminals in a graph of rich families can be connected: power to power with overlapping exact envelopes, power to ground, or signal to signal (lookup plus decision).",
    );
    for (kind, left, right) in [
        ("power-power", "vcc", "vcc"),
        ("power-ground", "vcc", "gnd"),
        ("signal-signal", "io", "io"),
    ] {
        atlas.sweep(
            kind,
            &families,
            |n, rng| {
                let n = n as usize;
                let (a, b) = (rng.below(n as u64) as usize, rng.below(n as u64) as usize);
                (graph(n, true), a, b)
            },
            |(graph, a, b)| {
                let variant = variant_id();
                graph
                    .safe_connection(
                        (&part_id(*a), &variant, &terminal_id(left)),
                        (&part_id(*b), &variant, &terminal_id(right)),
                    )
                    .map(|result| result.value)
            },
        );
    }

    // Exact bounds may cache approximations, so each call gets fresh ones.
    atlas.reuse_inputs(false);
    let bits = powers_of_two(0, 12);
    atlas.operation(
        "exact facts",
        "safe_connection by voltage bits",
        "bound operand bits",
        "Power-to-power decision for envelopes [a, a + b] and [a + b / 2, a + 2b] whose bounds have operands of growing size; an Unknown decision is marked as a failure.",
    );
    for kind in ["integer", "rational", "irrational"] {
        atlas.sweep(
            kind,
            &bits,
            |m, rng| {
                let (a, b) = (bound(kind, m as u32, rng), bound(kind, m as u32, rng));
                let half = (b.clone() / Real::from(2)).expect("nonzero divisor");
                let left = VoltageEnvelope::new(a.clone(), a.clone() + b.clone());
                let right = VoltageEnvelope::new(a.clone() + half, a + b.clone() + b);
                (left.ok(), right.ok())
            },
            |(left, right)| {
                let (Some(left), Some(right)) = (left, right) else {
                    return None;
                };
                decide(&connection_graph(left.clone(), right.clone()), 0, 1)
            },
        );
    }
    atlas.fuzz(
        "mixed kinds",
        &powers_of_two(0, 10),
        |m, rng| {
            let kinds = ["integer", "rational", "irrational"];
            let pick = |rng: &mut Rng| {
                let kind = kinds[rng.below(3) as usize];
                bound(kind, rng.range(1, m as i64) as u32, rng)
            };
            let (a, b, c, d) = (pick(rng), pick(rng), pick(rng), pick(rng));
            (
                VoltageEnvelope::new(a.clone(), a + b).ok(),
                VoltageEnvelope::new(c.clone(), c + d).ok(),
            )
        },
        |(left, right)| {
            let (Some(left), Some(right)) = (left, right) else {
                return None;
            };
            decide(&connection_graph(left.clone(), right.clone()), 0, 1)
        },
    );

    atlas.operation(
        "exact facts",
        "safe_connection near touching",
        "envelope gap exponent k (gap 2^-k)",
        "Decide [1, x] against [x + 2^-k, 10] (certified disjoint, Unsafe) for a rational, square-root or pi upper bound x; the touching case (gap 0) is the last magnitude. Unknown is marked as a failure.",
    );
    let gaps: Vec<f64> = powers_of_two(0, 12).into_iter().chain([8192.0]).collect();
    for kind in ["rational 7/3", "sqrt(2)", "pi"] {
        atlas.sweep(
            kind,
            &gaps,
            |k, _| {
                let x = match kind {
                    "rational 7/3" => (Real::from(7) / Real::from(3)).expect("nonzero"),
                    "sqrt(2)" => Real::from(2).sqrt().expect("positive"),
                    _ => Real::pi(),
                };
                // The last magnitude stands for "touching": the gap is zero.
                let gap = if k > 4096.0 {
                    Real::zero()
                } else {
                    Real::new(&Rational::new(1) / &pow2(k as u32))
                };
                let left = VoltageEnvelope::new(Real::from(1), x.clone());
                let right = VoltageEnvelope::new(x + gap, Real::from(10));
                (left.ok(), right.ok())
            },
            |(left, right)| {
                let (Some(left), Some(right)) = (left, right) else {
                    return None;
                };
                decide(&connection_graph(left.clone(), right.clone()), 0, 1)
            },
        );
    }

    atlas.operation(
        "exact facts",
        "interval validation",
        "bound operand bits",
        "Validate min <= max while constructing an exact voltage envelope and an exact assertion interval from bounds of growing size.",
    );
    for kind in ["integer", "rational", "irrational"] {
        atlas.sweep(
            kind,
            &bits,
            |m, rng| {
                let a = bound(kind, m as u32, rng);
                let b = bound(kind, m as u32, rng);
                (a.clone(), a + b)
            },
            |(min, max)| {
                let envelope = VoltageEnvelope::new(min.clone(), max.clone()).is_ok();
                let interval = AssertionValue::interval(min.clone(), max.clone()).is_ok();
                envelope && interval
            },
        );
    }

    let base = BundleShape {
        records: 4,
        digits: 4,
        pins: 4,
        parameters: 4,
        routes: 2,
    };
    atlas.operation(
        "eda intake",
        "import bundle by circuit records",
        "circuit records",
        "Import a tscircuit-style bundle whose circuit records carry two exact fields each (the bundle is cloned in the call); exact decimals, p/q rationals, malformed values (rejected) or omitted values (unknown).",
    );
    let records = powers_of_two(0, 13);
    for kind in FIELD_KINDS {
        atlas.sweep(
            kind,
            &records,
            |n, rng| {
                bundle(
                    kind,
                    &BundleShape {
                        records: n as usize,
                        ..base
                    },
                    rng,
                )
            },
            |bundle| intake(import_eda_authoring_bundle(bundle.clone())),
        );
    }
    atlas.operation(
        "eda intake",
        "import bundle by value digits",
        "decimal digits per value part",
        "Import a four-record bundle whose exact field values have growing digit counts (integer and fraction parts, or numerator and denominator).",
    );
    let digits = powers_of_two(0, 14);
    for kind in ["decimal", "rational"] {
        atlas.sweep(
            kind,
            &digits,
            |d, rng| {
                bundle(
                    kind,
                    &BundleShape {
                        digits: d as usize,
                        ..base
                    },
                    rng,
                )
            },
            |bundle| intake(import_eda_authoring_bundle(bundle.clone())),
        );
    }
    atlas.fuzz(
        "decimal",
        &ladder(1, 4096, 7),
        |d, rng| {
            let digits = rng.range(1, d as i64) as usize;
            let records = rng.range(1, 32) as usize;
            bundle(
                FIELD_KINDS[rng.below(4) as usize],
                &BundleShape {
                    records,
                    digits,
                    ..base
                },
                rng,
            )
        },
        |bundle| intake(import_eda_authoring_bundle(bundle.clone())),
    );
    atlas.operation(
        "eda intake",
        "import bundle by package size",
        "pins, footprint parameters or routes",
        "Import a bundle as one section grows: package pins with exact voltage bounds, numeric footprint parameters, or route and fabrication records.",
    );
    let sizes = powers_of_two(0, 13);
    for kind in ["package pins", "footprint parameters", "routes"] {
        atlas.sweep(
            kind,
            &sizes,
            |n, rng| {
                let n = n as usize;
                let shape = match kind {
                    "package pins" => BundleShape { pins: n, ..base },
                    "footprint parameters" => BundleShape {
                        parameters: n,
                        ..base
                    },
                    _ => BundleShape { routes: n, ..base },
                };
                bundle("decimal", &shape, rng)
            },
            |bundle| intake(import_eda_authoring_bundle(bundle.clone())),
        );
    }
    atlas.finish();
}
