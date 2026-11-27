//! Performance and resource atlas for Hypercircuit: netlist carriers, exact
//! MNA assembly and solving, nonlinear DC (diode, MOSFET, piecewise-linear),
//! transient and AC simulation, event agendas, hierarchy flattening, ERC,
//! schematics and KiCad schematic exchange, and the PCB layout workflow
//! (validation, placement, negotiated autorouting, KiCad PCB round trip and,
//! with `geometry`, copper/drill materialization).
//!
//! Axes:
//! - **node and element count** for assembly, solving, simulation and
//!   netlist operations, on ladder, 2D mesh, star and dense topologies;
//! - **scalar kind**: integer, rational and irrational (`sqrt`) element
//!   values, which drive exact-arithmetic growth in the eliminations;
//! - **horizon** (accepted timestep count) for transient runs, with
//!   non-stiff RC ladders (equal time constants) against stiff ones
//!   (alternating capacitances 1 and 10^-6, a 10^6 time-constant spread),
//!   under trapezoidal and first-order Gear (backward Euler) companions,
//!   fixed and step-doubling adaptive;
//! - **device count** for Newton and piecewise-linear region search;
//! - **frequency count** for AC sweeps; **event count** for agendas;
//!   **hierarchy depth** and instance count for flattening;
//! - **component, net and grid resolution** for layout and routing.
//!
//! In resource runs, solutions report their size: `nodes` is the unknown,
//! sample, instance or feature count, `depth` the largest expression depth of
//! a solved value (Newton iteration count for nonlinear solves; scope depth
//! for flattening), and `bits` the largest stored rational in any solved
//! value.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hypercircuit --all-features --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
use std::collections::BTreeMap;

use hypercircuit::{
    AcExcitation, AdapterKind, BoardId, BoardOutline, BoardSide, Circuit, CircuitEventAgenda,
    CircuitEventCause, CircuitEventKind, CircuitEventPhase, CircuitEventRequest,
    CircuitEventTarget, CircuitId, CircuitInstance, CircuitInstanceId, CircuitLibrary,
    CircuitParameter, CircuitPort, ComponentId, DeviceModel, DeviceModelId, DeviceModelKind,
    DevicePin, DiodeNewtonPolicy, DiodeNewtonStatus, KiCadExportOptions, KiCadImportOptions,
    KiCadImportReport, KiCadSchematicExportOptions, KiCadSchematicImportReport, LandPattern,
    LandPatternId, LandPatternPad, LinearMnaSystem, LinearSolveReport, LinearStamp,
    MosfetNewtonPolicy, MosfetNewtonStatus, MosfetPolarity, NegotiatedRoutePolicy,
    NegotiatedRouteStatus, Net, NetId, PadId, PadPinMap, PadShape, PcbLayout, PcbPlacement,
    PcbStackup, Phasor, PiecewiseLinearDevice, PiecewiseLinearSegment, PinBinding,
    PinElectricalKind, PinRef, PlacementSolvePolicy, Plating, PortDirection, PortId, Real,
    RoutingProblemReport, SchematicAutoLayoutPolicy, SourceStimulus, SourceWaveform,
    SubcircuitInstance, SubcircuitInstanceId, SubcircuitPortBinding, TransientAdaptation,
    TransientHistory, TransientPolicy, TransientRunPolicy, TransientRunReport, TransientRunStatus,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;
use perf_atlas::{Atlas, Checked, Rng, Shape, Shaped, ladder, powers_of_two};

#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

fn r(value: i64) -> Real {
    Real::from(value)
}

fn q(numerator: i64, denominator: i64) -> Real {
    (r(numerator) / r(denominator)).expect("nonzero denominator")
}

fn net(name: impl Into<String>) -> NetId {
    NetId::new(name).expect("valid net id")
}

/// Largest expression depth and stored rational over solved values.
fn value_shape<'a>(values: impl IntoIterator<Item = &'a Real>) -> (u64, u64, u64) {
    let mut count = 0;
    let mut depth = 0;
    let mut bits = 0;
    for value in values {
        let size = value.representation_size(1 << 12);
        count += 1;
        depth = depth.max(size.depth as u64);
        bits = bits.max(size.max_rational_bits);
    }
    (count, depth, bits)
}

fn solve_shape(report: &Checked<LinearSolveReport>) -> Option<Shape> {
    let report = report.as_ref().ok()?;
    let (nodes, depth, bits) = value_shape(&report.candidate);
    Some(Shape { nodes, depth, bits })
}

fn checked<T, E: std::fmt::Debug>(result: Result<T, E>) -> Checked<T> {
    result.map_err(|error| format!("{error:?}"))
}

// ---------------------------------------------------------------------------
// Netlist construction
// ---------------------------------------------------------------------------

fn pin(name: &str) -> DevicePin {
    DevicePin {
        pin: PinRef::new(name).unwrap(),
        kind: PinElectricalKind::Passive,
        optional: false,
    }
}

fn parameter(name: &str, value: Real) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value,
        unit: String::new(),
        source: "perf-atlas".into(),
    }
}

/// A circuit under construction with shared two-terminal models
/// (`R`, `C`, `L`, `V`, `I`, `D`) and an n-channel square-law MOSFET (`M`).
struct Netlist {
    circuit: Circuit,
    instances: usize,
}

impl Netlist {
    fn new(name: &str, policy: TransientPolicy, adapter: AdapterKind) -> Self {
        let two = || vec![pin("+"), pin("-")];
        let model = |id: &str, kind: DeviceModelKind, pins, parameters| DeviceModel {
            id: DeviceModelId::new(id).unwrap(),
            kind,
            pins,
            parameters,
        };
        let circuit = Circuit::new(CircuitId::new(name).unwrap(), policy, adapter)
            .with_net(Net {
                id: net("GND"),
                is_ground: true,
            })
            .with_device_model(model(
                "R",
                DeviceModelKind::Resistor,
                two(),
                vec![parameter("resistance", r(1))],
            ))
            .with_device_model(model(
                "C",
                DeviceModelKind::Capacitor,
                two(),
                vec![parameter("capacitance", r(1))],
            ))
            .with_device_model(model(
                "L",
                DeviceModelKind::Inductor,
                two(),
                vec![parameter("inductance", r(1))],
            ))
            .with_device_model(model(
                "V",
                DeviceModelKind::VoltageSource,
                two(),
                vec![parameter("voltage", r(1))],
            ))
            .with_device_model(model(
                "I",
                DeviceModelKind::CurrentSource,
                two(),
                vec![parameter("current", r(1))],
            ))
            .with_device_model(model(
                "D",
                DeviceModelKind::Diode,
                two(),
                vec![
                    parameter("saturation_current", q(1, 10)),
                    parameter("thermal_voltage", q(1, 2)),
                ],
            ))
            .with_device_model(model(
                "M",
                DeviceModelKind::Mosfet {
                    polarity: MosfetPolarity::NChannel,
                    drain: PinRef::new("D").unwrap(),
                    gate: PinRef::new("G").unwrap(),
                    source: PinRef::new("S").unwrap(),
                },
                vec![pin("D"), pin("G"), pin("S")],
                vec![
                    parameter("threshold_voltage", r(1)),
                    parameter("transconductance_parameter", r(1)),
                    parameter("channel_length_modulation", r(0)),
                ],
            ));
        Self {
            circuit,
            instances: 0,
        }
    }

    fn node(&mut self, name: impl Into<String>) -> NetId {
        let id = net(name);
        self.circuit.nets.push(Net {
            id: id.clone(),
            is_ground: false,
        });
        id
    }

    /// Add one element; `pins` pair model pin names with nets.
    fn add(
        &mut self,
        model: &str,
        pins: &[(&str, &NetId)],
        parameters: Vec<(&str, Real)>,
    ) -> ComponentId {
        self.instances += 1;
        let name = format!("{model}{}", self.instances);
        let component = ComponentId::new(name.clone()).unwrap();
        self.circuit.instances.push(CircuitInstance {
            id: CircuitInstanceId::new(name).unwrap(),
            component: component.clone(),
            part: None,
            model: DeviceModelId::new(model).unwrap(),
            pins: pins
                .iter()
                .map(|(pin, net)| PinBinding {
                    pin: PinRef::new(*pin).unwrap(),
                    net: (*net).clone(),
                })
                .collect(),
            parameters: parameters
                .into_iter()
                .map(|(name, value)| parameter(name, value))
                .collect(),
        });
        component
    }

    fn two(&mut self, model: &str, pos: &NetId, neg: &NetId, parameters: Vec<(&str, Real)>) {
        self.add(model, &[("+", pos), ("-", neg)], parameters);
    }
}

/// Element values: the scalar kind of a netlist.
#[derive(Clone, Copy)]
enum Scalar {
    Integer,
    Rational,
    Irrational,
}

const SCALARS: [(&str, Scalar); 3] = [
    ("integer values", Scalar::Integer),
    ("rational values", Scalar::Rational),
    ("irrational values", Scalar::Irrational),
];

fn value(kind: Scalar, rng: &mut Rng) -> Real {
    match kind {
        Scalar::Integer => r(rng.range(1, 9)),
        Scalar::Rational => q(rng.range(1, 97), rng.range(1, 97)),
        Scalar::Irrational => r(rng.range(2, 9)).sqrt().expect("positive square root"),
    }
}

/// Topologies for linear netlists, as a function of the node count `n`.
const TOPOLOGIES: [&str; 3] = ["ladder", "2D mesh", "star"];

/// A driven resistor network of about `n` non-ground nodes.
fn resistor_network(topology: &str, n: usize, scalar: Scalar, rng: &mut Rng) -> Circuit {
    let mut list = Netlist::new("network", TransientPolicy::Static, AdapterKind::Dc);
    let gnd = net("GND");
    let source = list.node("in");
    list.two("V", &source, &gnd, vec![("voltage", r(5))]);
    match topology {
        "2D mesh" => {
            let side = (n as f64).sqrt().ceil().max(1.0) as usize;
            let nodes: Vec<NetId> = (0..side * side)
                .map(|i| list.node(format!("m{i}")))
                .collect();
            list.two(
                "R",
                &source,
                &nodes[0],
                vec![("resistance", value(scalar, rng))],
            );
            for row in 0..side {
                for col in 0..side {
                    let here = &nodes[row * side + col];
                    if col + 1 < side {
                        let right = nodes[row * side + col + 1].clone();
                        list.two("R", here, &right, vec![("resistance", value(scalar, rng))]);
                    }
                    if row + 1 < side {
                        let down = nodes[(row + 1) * side + col].clone();
                        list.two("R", here, &down, vec![("resistance", value(scalar, rng))]);
                    }
                }
            }
            let last = nodes[side * side - 1].clone();
            list.two("R", &last, &gnd, vec![("resistance", value(scalar, rng))]);
        }
        "star" => {
            let hub = list.node("hub");
            list.two("R", &source, &hub, vec![("resistance", value(scalar, rng))]);
            for i in 0..n.saturating_sub(1).max(1) {
                let leaf = list.node(format!("leaf{i}"));
                list.two("R", &hub, &leaf, vec![("resistance", value(scalar, rng))]);
                list.two("R", &leaf, &gnd, vec![("resistance", value(scalar, rng))]);
            }
        }
        _ => {
            let mut previous = source;
            for i in 0..n.max(1) {
                let node = list.node(format!("n{i}"));
                list.two(
                    "R",
                    &previous,
                    &node,
                    vec![("resistance", value(scalar, rng))],
                );
                list.two("R", &node, &gnd, vec![("resistance", value(scalar, rng))]);
                previous = node;
            }
        }
    }
    list.circuit
}

/// Raw conductance stamps on `n` nets: ladder, 2D mesh or dense (every pair
/// coupled), driven by one voltage source and grounded through shunts.
fn stamps(
    topology: &str,
    n: usize,
    scalar: Scalar,
    rng: &mut Rng,
) -> (Vec<NetId>, Vec<LinearStamp>) {
    let n = n.max(1);
    let nets: Vec<NetId> = (0..n).map(|i| net(format!("n{i}"))).collect();
    let mut stamps = vec![LinearStamp::VoltageSource {
        component: ComponentId::new("V1").unwrap(),
        branch: hypercircuit::BranchId::new("iV1").unwrap(),
        pos: Some(nets[0].clone()),
        neg: None,
        voltage: r(5),
    }];
    let mut count = 0;
    let mut conductance = |a: Option<&NetId>, b: Option<&NetId>, rng: &mut Rng| {
        count += 1;
        LinearStamp::Conductance {
            component: ComponentId::new(format!("G{count}")).unwrap(),
            part: None,
            pos: a.cloned(),
            neg: b.cloned(),
            conductance: value(scalar, rng),
        }
    };
    for node in &nets {
        stamps.push(conductance(Some(node), None, rng));
    }
    match topology {
        "dense" => {
            for i in 0..n {
                for j in (i + 1)..n {
                    stamps.push(conductance(Some(&nets[i]), Some(&nets[j]), rng));
                }
            }
        }
        "2D mesh" => {
            let side = (n as f64).sqrt().ceil() as usize;
            for i in 0..n {
                if (i + 1) % side != 0 && i + 1 < n {
                    stamps.push(conductance(Some(&nets[i]), Some(&nets[i + 1]), rng));
                }
                if i + side < n {
                    stamps.push(conductance(Some(&nets[i]), Some(&nets[i + side]), rng));
                }
            }
        }
        _ => {
            for i in 1..n {
                stamps.push(conductance(Some(&nets[i - 1]), Some(&nets[i]), rng));
            }
        }
    }
    (nets, stamps)
}

/// Stiffness of an RC ladder.
#[derive(Clone, Copy, PartialEq)]
enum Stiffness {
    /// Every stage has R = C = 1.
    NonStiff,
    /// Alternating C = 1 and C = 10^-6: a 10^6 spread of time constants.
    Stiff,
}

/// A constant-voltage-driven RC ladder of `stages` stages.
fn rc_ladder(stages: usize, stiffness: Stiffness, policy: TransientPolicy) -> Circuit {
    let mut list = Netlist::new("rc-ladder", policy, AdapterKind::TransientDae);
    let gnd = net("GND");
    let mut previous = list.node("in");
    list.two("V", &previous, &gnd, vec![("voltage", r(1))]);
    for i in 0..stages.max(1) {
        let node = list.node(format!("n{i}"));
        list.two("R", &previous, &node, vec![]);
        let capacitance = if stiffness == Stiffness::Stiff && i % 2 == 1 {
            q(1, 1_000_000)
        } else {
            r(1)
        };
        list.two("C", &node, &gnd, vec![("capacitance", capacitance)]);
        previous = node;
    }
    list.circuit
}

/// An AC-driven RC (or series-RLC) ladder of `stages` stages.
fn ac_ladder(stages: usize, inductive: bool) -> Circuit {
    let mut list = Netlist::new("ac-ladder", TransientPolicy::Static, AdapterKind::Ac);
    let gnd = net("GND");
    let mut previous = list.node("in");
    list.two("V", &previous, &gnd, vec![("voltage", r(0))]);
    for i in 0..stages.max(1) {
        let node = list.node(format!("n{i}"));
        if inductive {
            let middle = list.node(format!("l{i}"));
            list.two("R", &previous, &middle, vec![]);
            list.two("L", &middle, &node, vec![]);
        } else {
            list.two("R", &previous, &node, vec![]);
        }
        list.two("C", &node, &gnd, vec![]);
        previous = node;
    }
    list.circuit
}

/// `count` diodes, as a ladder (each node feeds the next through a resistor
/// and shunts to ground through a diode) or as independent parallel branches.
fn diode_network(count: usize, cascade: bool) -> Circuit {
    let mut list = Netlist::new("diodes", TransientPolicy::Static, AdapterKind::Dc);
    let gnd = net("GND");
    let supply = list.node("vdd");
    list.two("V", &supply, &gnd, vec![("voltage", r(5))]);
    let mut previous = supply.clone();
    for i in 0..count.max(1) {
        let node = list.node(format!("d{i}"));
        let feed = if cascade {
            previous.clone()
        } else {
            supply.clone()
        };
        list.two("R", &feed, &node, vec![]);
        list.two("D", &node, &gnd, vec![]);
        previous = node;
    }
    list.circuit
}

/// `count` common-source stages sharing a supply; a cascade drives each gate
/// from the previous drain, parallel stages share one gate bias.
fn mosfet_network(count: usize, cascade: bool) -> Circuit {
    let mut list = Netlist::new("mosfets", TransientPolicy::Static, AdapterKind::Dc);
    let gnd = net("GND");
    let supply = list.node("vdd");
    let bias = list.node("bias");
    list.two("V", &supply, &gnd, vec![("voltage", r(5))]);
    list.two("V", &bias, &gnd, vec![("voltage", r(2))]);
    let mut gate = bias.clone();
    for i in 0..count.max(1) {
        let drain = list.node(format!("drain{i}"));
        list.two("R", &supply, &drain, vec![]);
        list.add("M", &[("D", &drain), ("G", &gate), ("S", &gnd)], vec![]);
        if cascade {
            gate = drain;
        }
    }
    list.circuit
}

/// A breakdown-like piecewise-linear device per node: `i = v/1000` up to 1 V
/// and slope 1 above it, driven from a 5 V supply through resistors.
fn piecewise_problem(count: usize) -> (Vec<NetId>, Vec<LinearStamp>, Vec<PiecewiseLinearDevice>) {
    let (nets, stamps) = stamps("ladder", count, Scalar::Integer, &mut Rng::new(1));
    let devices = nets
        .iter()
        .enumerate()
        .map(|(i, node)| PiecewiseLinearDevice {
            component: ComponentId::new(format!("P{i}")).unwrap(),
            pos: Some(node.clone()),
            neg: None,
            segments: vec![
                PiecewiseLinearSegment {
                    lower: r(-1000),
                    upper: r(1),
                    slope: q(1, 1000),
                    intercept: r(0),
                },
                PiecewiseLinearSegment {
                    lower: r(1),
                    upper: r(1000),
                    slope: r(1),
                    intercept: q(1, 1000) - r(1),
                },
            ],
        })
        .collect();
    (nets, stamps, devices)
}

fn fixed_policy(steps: usize, timestep: Real) -> TransientRunPolicy {
    TransientRunPolicy {
        start_time: r(0),
        stop_time: timestep.clone() * r(steps as i64),
        initial_timestep: timestep.clone(),
        minimum_timestep: timestep.clone(),
        maximum_timestep: timestep,
        maximum_accepted_steps: steps + 1,
        maximum_rejected_steps: 1,
        adaptation: TransientAdaptation::Fixed,
    }
}

fn adaptive_policy(stop: i64) -> TransientRunPolicy {
    TransientRunPolicy {
        start_time: r(0),
        stop_time: r(stop),
        initial_timestep: q(1, 8),
        minimum_timestep: q(1, 1 << 24),
        maximum_timestep: r(1),
        maximum_accepted_steps: 100_000,
        maximum_rejected_steps: 10_000,
        adaptation: TransientAdaptation::StepDoubling {
            absolute_tolerance: q(1, 1000),
            relative_tolerance: q(1, 100),
            shrink_factor: q(1, 2),
            growth_factor: r(2),
        },
    }
}

fn transient_checked(
    report: Result<TransientRunReport, impl std::fmt::Debug>,
) -> Shaped<Checked<TransientRunReport>> {
    let report = checked(report).and_then(|report| match report.status {
        TransientRunStatus::Complete => Ok(report),
        status => Err(format!(
            "{status:?} after {} accepted, {} attempts",
            report.samples.len(),
            report.decisions.len()
        )),
    });
    Shaped::new(report, |report| {
        let report = report.as_ref().ok()?;
        let last = report.samples.last()?;
        let (_, depth, bits) = value_shape(last.unknowns.values());
        Some(Shape {
            nodes: report.decisions.len() as u64,
            depth,
            bits,
        })
    })
}

const STIFF_KINDS: [(&str, Stiffness, TransientPolicy); 3] = [
    (
        "non-stiff, trapezoidal",
        Stiffness::NonStiff,
        TransientPolicy::Trapezoidal,
    ),
    (
        "stiff, trapezoidal",
        Stiffness::Stiff,
        TransientPolicy::Trapezoidal,
    ),
    (
        "stiff, Gear BDF1",
        Stiffness::Stiff,
        TransientPolicy::GearBdf { order: 1 },
    ),
];

// ---------------------------------------------------------------------------
// Operation groups
// ---------------------------------------------------------------------------

fn netlist_ops(atlas: &mut Atlas) {
    let counts = powers_of_two(1, 12);
    atlas.operation(
        "netlist",
        "Circuit::validate",
        "node count",
        "Structural validation of a driven resistor network (identity, pin binding and model checks).",
    );
    for topology in TOPOLOGIES {
        atlas.sweep(
            topology,
            &counts,
            |n, rng| resistor_network(topology, n as usize, Scalar::Integer, rng),
            |c| c.validate().is_valid(),
        );
    }
    atlas.operation(
        "netlist",
        "lower_linear_devices",
        "node count",
        "Validation plus lowering of every primitive device to linear MNA stamps.",
    );
    for topology in TOPOLOGIES {
        atlas.sweep(
            topology,
            &counts,
            |n, rng| resistor_network(topology, n as usize, Scalar::Integer, rng),
            |c| {
                let report = c.lower_linear_devices();
                report.is_complete().then_some(report.stamps.len())
            },
        );
    }
    atlas.operation(
        "netlist",
        "electrical_rule_check",
        "node count",
        "Pre-layout ERC over every net and pin of a resistor network.",
    );
    for topology in TOPOLOGIES {
        atlas.sweep(
            topology,
            &counts,
            |n, rng| resistor_network(topology, n as usize, Scalar::Integer, rng),
            |c| c.electrical_rule_check().issues.len(),
        );
    }

    // Hierarchy: a resistor leaf module instantiated flat, as a binary tree
    // and as a chain (depth = instance count).
    atlas.operation(
        "netlist",
        "CircuitLibrary::flatten",
        "leaf instances",
        "Elaboration of reusable subcircuits into one flat circuit with path-qualified identities; shapes report flat instances and scope depth.",
    );
    for kind in ["flat fan-out", "binary tree", "chain"] {
        let range = if kind == "chain" {
            powers_of_two(0, 9)
        } else {
            powers_of_two(0, 11)
        };
        atlas.sweep(
            kind,
            &range,
            |n, _| hierarchy(kind, n as usize),
            |library| {
                Shaped::new(checked(library.flatten_with_scopes()), |report| {
                    report.as_ref().ok().map(|report| Shape {
                        nodes: report.circuit.instances.len() as u64,
                        depth: report
                            .scopes
                            .iter()
                            .map(|scope| scope.path.len() as u64)
                            .max()
                            .unwrap_or(0),
                        bits: 0,
                    })
                })
            },
        );
    }

    atlas.operation(
        "schematic",
        "auto_schematic",
        "node count",
        "Connectivity-driven generic schematic: breadth-first columns, symbols, wires and labels.",
    );
    for topology in TOPOLOGIES {
        atlas.sweep(
            topology,
            &powers_of_two(1, 11),
            |n, rng| resistor_network(topology, n as usize, Scalar::Integer, rng),
            |c| {
                checked(c.auto_schematic(SchematicAutoLayoutPolicy::default()))
                    .map(|report| report.generated_wires)
            },
        );
    }
    atlas.operation(
        "schematic",
        "export_kicad_schematic",
        "node count",
        "Native KiCad 9 schematic text for an auto-generated drawing (drawing built outside the timer).",
    );
    for topology in TOPOLOGIES {
        atlas.sweep(
            topology,
            &powers_of_two(1, 10),
            |n, rng| {
                let c = resistor_network(topology, n as usize, Scalar::Integer, rng);
                let layout = c
                    .auto_schematic(SchematicAutoLayoutPolicy::default())
                    .expect("auto schematic")
                    .layout;
                (c, layout)
            },
            |(c, layout)| {
                checked(layout.export_kicad_schematic(c, KiCadSchematicExportOptions::default()))
                    .map(|report| report.schematic.len())
            },
        );
    }
    atlas.operation(
        "schematic",
        "KiCadSchematicImportReport::from_str",
        "node count",
        "Parse and bind an exported KiCad schematic back against circuit truth.",
    );
    for topology in TOPOLOGIES {
        atlas.sweep(
            topology,
            &powers_of_two(1, 10),
            |n, rng| {
                let c = resistor_network(topology, n as usize, Scalar::Integer, rng);
                let text = c
                    .auto_schematic(SchematicAutoLayoutPolicy::default())
                    .expect("auto schematic")
                    .layout
                    .export_kicad_schematic(&c, KiCadSchematicExportOptions::default())
                    .expect("export")
                    .schematic;
                (c, text)
            },
            |(c, text)| checked(KiCadSchematicImportReport::from_str(text, c)).map(|_| ()),
        );
    }

    atlas.operation(
        "events",
        "CircuitEventAgenda schedule+deliver",
        "event count",
        "Schedule timer events (in time order in bulk, or one by one in shuffled order) and deliver all of them.",
    );
    for kind in ["preordered bulk", "shuffled one by one"] {
        atlas.sweep(
            kind,
            &powers_of_two(1, 16),
            |n, rng| {
                let mut times: Vec<u64> = (0..n as u64).collect();
                if kind.starts_with("shuffled") {
                    for i in (1..times.len()).rev() {
                        let j = rng.below(i as u64 + 1) as usize;
                        times.swap(i, j);
                    }
                }
                times
            },
            |times| {
                let mut agenda = CircuitEventAgenda::new(r(0));
                let requests = times.iter().map(|&t| event(t));
                let scheduled = if times.windows(2).all(|w| w[0] <= w[1]) {
                    agenda.schedule_preordered(requests).map(|ids| ids.len())
                } else {
                    requests
                        .map(|request| agenda.schedule(request))
                        .try_fold(0, |n, id| id.map(|_| n + 1))
                };
                let mut delivered = 0;
                while agenda.deliver_next().is_some() {
                    delivered += 1;
                }
                checked(scheduled).and_then(|scheduled| {
                    (scheduled == delivered)
                        .then_some(delivered)
                        .ok_or_else(|| format!("{delivered} of {scheduled} delivered"))
                })
            },
        );
    }
}

fn event(time: u64) -> CircuitEventRequest {
    CircuitEventRequest {
        time: r(time as i64),
        phase: CircuitEventPhase::Stimulus,
        source: None,
        target: CircuitEventTarget::Circuit(CircuitId::new("atlas").unwrap()),
        kind: CircuitEventKind::Timer {
            key: format!("t{time}"),
        },
        cause: CircuitEventCause::Authored {
            provenance: "perf-atlas".into(),
        },
    }
}

/// A library whose root elaborates `leaves` resistor-divider leaf modules.
fn hierarchy(kind: &str, leaves: usize) -> CircuitLibrary {
    let port = |circuit: Circuit, name: &str, net_id: &NetId| {
        circuit.with_port(CircuitPort {
            id: PortId::new(name).unwrap(),
            net: net_id.clone(),
            direction: PortDirection::Passive,
            optional: false,
        })
    };
    let instance = |id: String, child: &CircuitId, a: &NetId, b: &NetId| SubcircuitInstance {
        id: SubcircuitInstanceId::new(id).unwrap(),
        circuit: child.clone(),
        ports: vec![
            SubcircuitPortBinding {
                port: PortId::new("a").unwrap(),
                net: a.clone(),
            },
            SubcircuitPortBinding {
                port: PortId::new("b").unwrap(),
                net: b.clone(),
            },
        ],
        parameter_overrides: Vec::new(),
    };
    // Leaf: a - R - mid - R - b.
    let mut leaf = Netlist::new("leaf", TransientPolicy::Static, AdapterKind::Dc);
    let (a, b) = (leaf.node("a"), leaf.node("b"));
    let mid = leaf.node("mid");
    leaf.two("R", &a, &mid, vec![]);
    leaf.two("R", &mid, &b, vec![]);
    // Modules connect only through ports; the root owns the ground net.
    leaf.circuit.nets.retain(|net| !net.is_ground);
    let leaf_circuit = port(port(leaf.circuit, "a", &a), "b", &b);
    let mut circuits = vec![leaf_circuit];
    let mut child = CircuitId::new("leaf").unwrap();
    let mut child_leaves = 1;
    let level = |name: String, parts: usize, child: &CircuitId| {
        let mut list = Netlist::new(&name, TransientPolicy::Static, AdapterKind::Dc);
        let (a, b) = (list.node("a"), list.node("b"));
        list.circuit.nets.retain(|net| !net.is_ground);
        let mut circuit = port(port(list.circuit, "a", &a), "b", &b);
        for i in 0..parts {
            circuit = circuit.with_subcircuit(instance(format!("u{i}"), child, &a, &b));
        }
        circuit
    };
    match kind {
        "binary tree" => {
            let mut depth = 0;
            while child_leaves * 2 <= leaves {
                depth += 1;
                let circuit = level(format!("tree{depth}"), 2, &child);
                child = circuit.id.clone();
                circuits.push(circuit);
                child_leaves *= 2;
            }
        }
        "chain" => {
            for depth in 1..leaves {
                let circuit = level(format!("chain{depth}"), 1, &child);
                child = circuit.id.clone();
                circuits.push(circuit);
            }
        }
        _ => {}
    }
    let parts = if kind == "flat fan-out" { leaves } else { 1 };
    let mut root = Netlist::new("root", TransientPolicy::Static, AdapterKind::Dc);
    let (a, b) = (root.node("a"), root.node("b"));
    let gnd = net("GND");
    root.two("V", &a, &gnd, vec![]);
    root.two("R", &b, &gnd, vec![]);
    let mut root_circuit = root.circuit;
    for i in 0..parts {
        root_circuit = root_circuit.with_subcircuit(instance(format!("top{i}"), &child, &a, &b));
    }
    let root_id = root_circuit.id.clone();
    circuits.push(root_circuit);
    CircuitLibrary {
        root: root_id,
        circuits,
    }
}

fn mna_ops(atlas: &mut Atlas) {
    let counts = powers_of_two(1, 10);
    atlas.operation(
        "mna",
        "LinearMnaSystem::from_stamps",
        "node count",
        "Dense exact MNA assembly (unknown ordering, conductance and source stamps) from raw stamps.",
    );
    for topology in ["ladder", "2D mesh", "dense"] {
        atlas.sweep(
            topology,
            &counts,
            |n, rng| stamps(topology, n as usize, Scalar::Rational, rng),
            |(nets, stamps)| {
                checked(LinearMnaSystem::from_stamps(nets.clone(), stamps))
                    .map(|s| s.unknowns.len())
            },
        );
    }
    atlas.operation(
        "mna",
        "linear_mna_from_devices",
        "node count",
        "Validation, device lowering and dense assembly from a retained netlist.",
    );
    for topology in TOPOLOGIES {
        atlas.sweep(
            topology,
            &counts,
            |n, rng| resistor_network(topology, n as usize, Scalar::Rational, rng),
            |c| checked(c.linear_mna_from_devices()).map(|s| s.unknowns.len()),
        );
    }
    atlas.operation(
        "mna",
        "solve_exact (ladder)",
        "unknowns",
        "Exact-aware Gauss-Jordan solve with certified pivots and residual replay, on a ladder (tridiagonal) system; series by element value kind.",
    );
    for (kind, scalar) in SCALARS {
        atlas.sweep(
            kind,
            &powers_of_two(1, 9),
            |n, rng| {
                let (nets, stamps) = stamps("ladder", n as usize, scalar, rng);
                LinearMnaSystem::from_stamps(nets, &stamps).expect("assembly")
            },
            |system| Shaped::new(checked(system.solve_exact()), solve_shape),
        );
    }
    for topology in ["2D mesh", "dense"] {
        atlas.operation(
            "mna",
            &format!("solve_exact ({topology})"),
            "unknowns",
            "Exact-aware Gauss-Jordan solve with residual replay; series by element value kind.",
        );
        for (kind, scalar) in SCALARS {
            atlas.sweep(
                kind,
                &powers_of_two(1, 8),
                |n, rng| {
                    let (nets, stamps) = stamps(topology, n as usize, scalar, rng);
                    LinearMnaSystem::from_stamps(nets, &stamps).expect("assembly")
                },
                |system| Shaped::new(checked(system.solve_exact()), solve_shape),
            );
        }
    }
    atlas.operation(
        "mna",
        "solve_exact fuzz",
        "unknowns",
        "Random rational 2D-mesh systems: spread of exact elimination cost.",
    );
    atlas.fuzz(
        "rational 2D mesh",
        &powers_of_two(2, 6),
        |n, rng| {
            let (nets, stamps) = stamps("2D mesh", n as usize, Scalar::Rational, rng);
            LinearMnaSystem::from_stamps(nets, &stamps).expect("assembly")
        },
        |system| Shaped::new(checked(system.solve_exact()), solve_shape),
    );
    atlas.operation(
        "mna",
        "replay_candidate",
        "unknowns",
        "Exact residual replay of the solved candidate on a 2D mesh (solve outside the timer, so sizes stop where the exact solve becomes slow; irrational meshes are omitted because their solve is refused).",
    );
    for (kind, scalar, top) in [
        ("integer values", Scalar::Integer, 6),
        ("rational values", Scalar::Rational, 5),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(1, top),
            |n, rng| {
                let (nets, stamps) = stamps("2D mesh", n as usize, scalar, rng);
                let system = LinearMnaSystem::from_stamps(nets, &stamps).expect("assembly");
                let candidate = system
                    .solve_exact()
                    .map(|report| report.candidate)
                    .unwrap_or_default();
                (system, candidate)
            },
            |(system, candidate)| {
                checked(system.replay_candidate(candidate)).and_then(|replay| {
                    replay
                        .accepted
                        .then_some(())
                        .ok_or_else(|| "not accepted".to_owned())
                })
            },
        );
    }
}

fn nonlinear_ops(atlas: &mut Atlas) {
    atlas.operation(
        "nonlinear dc",
        "solve_diode_dc",
        "diode count",
        "Damped Newton on Shockley diodes (default policy, tolerance 10^-9) with true-law exact replay; depth is the Newton iteration count.",
    );
    for (kind, cascade) in [("diode ladder", true), ("parallel branches", false)] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 7),
            |n, _| diode_network(n as usize, cascade),
            |c| {
                let report =
                    checked(c.solve_diode_dc(&DiodeNewtonPolicy::default(), &BTreeMap::new()))
                        .and_then(|report| match report.status {
                            DiodeNewtonStatus::Converged => Ok(report),
                            status => Err(format!("{status:?}")),
                        });
                Shaped::new(report, |report| {
                    let report = report.as_ref().ok()?;
                    let (nodes, _, bits) = value_shape(&report.candidate);
                    Some(Shape {
                        nodes,
                        depth: report.iterations.len() as u64,
                        bits,
                    })
                })
            },
        );
    }
    atlas.operation(
        "nonlinear dc",
        "solve_mosfet_dc",
        "stage count",
        "Damped Newton on square-law MOSFET common-source stages with exact true-law replay; depth is the Newton iteration count.",
    );
    for (kind, cascade) in [("parallel stages", false), ("cascade", true)] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 6),
            |n, _| mosfet_network(n as usize, cascade),
            |c| {
                let report =
                    checked(c.solve_mosfet_dc(&MosfetNewtonPolicy::default(), &BTreeMap::new()))
                        .and_then(|report| match report.status {
                            MosfetNewtonStatus::Converged => Ok(report),
                            status => Err(format!("{status:?}")),
                        });
                Shaped::new(report, |report| {
                    let report = report.as_ref().ok()?;
                    let (nodes, _, bits) = value_shape(&report.candidate);
                    Some(Shape {
                        nodes,
                        depth: report.iterations.len() as u64,
                        bits,
                    })
                })
            },
        );
    }
    atlas.operation(
        "nonlinear dc",
        "solve_piecewise_linear",
        "device count",
        "Exact active-region enumeration for two-segment piecewise-linear devices on a ladder (2^devices combinations, each an exact solve).",
    );
    atlas.sweep(
        "breakdown devices",
        &ladder(1, 16, 16),
        |n, _| piecewise_problem(n as usize),
        |(nets, stamps, devices)| {
            checked(hypercircuit::solve_piecewise_linear(
                nets.clone(),
                stamps,
                devices,
                1 << 20,
            ))
            .map(|report| report.candidates_tried)
        },
    );
}

fn transient_ops(atlas: &mut Atlas) {
    let steps = powers_of_two(0, 10);
    atlas.operation(
        "transient",
        "transient_run fixed (horizon)",
        "accepted steps",
        "Fixed-step exact companion transient of a 4-stage RC ladder, timestep 1/4; each step assembles, solves and replays exactly. Shapes report attempts and the largest stored rational in the final state.",
    );
    for (kind, stiffness, policy) in STIFF_KINDS {
        atlas.sweep(
            kind,
            &steps,
            |n, _| {
                (
                    rc_ladder(4, stiffness, policy.clone()),
                    fixed_policy(n as usize, q(1, 4)),
                )
            },
            |(c, run)| transient_checked(c.transient_run(run, TransientHistory::default())),
        );
    }
    atlas.operation(
        "transient",
        "transient_run fixed (nodes)",
        "ladder stages",
        "Sixteen fixed exact companion steps (timestep 1/4) on RC ladders of growing size.",
    );
    for (kind, stiffness, policy) in STIFF_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 8),
            |n, _| {
                (
                    rc_ladder(n as usize, stiffness, policy.clone()),
                    fixed_policy(16, q(1, 4)),
                )
            },
            |(c, run)| transient_checked(c.transient_run(run, TransientHistory::default())),
        );
    }
    atlas.grid(
        "non-stiff, trapezoidal",
        ("ladder stages", &powers_of_two(0, 6)),
        ("accepted steps", &powers_of_two(0, 8)),
        |n, s, _| {
            (
                rc_ladder(
                    n as usize,
                    Stiffness::NonStiff,
                    TransientPolicy::Trapezoidal,
                ),
                fixed_policy(s as usize, q(1, 4)),
            )
        },
        |(c, run)| transient_checked(c.transient_run(run, TransientHistory::default())),
    );
    atlas.operation(
        "transient",
        "transient_run adaptive",
        "stop time",
        "Step-doubling adaptive transient (absolute 10^-3, relative 10^-2, halving on rejection) of a 4-stage RC ladder up to the stop time; nodes count every accepted and rejected attempt.",
    );
    for (kind, stiffness, policy) in STIFF_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 6),
            |t, _| {
                (
                    rc_ladder(4, stiffness, policy.clone()),
                    adaptive_policy(t as i64),
                )
            },
            |(c, run)| transient_checked(c.transient_run(run, TransientHistory::default())),
        );
    }
    atlas.operation(
        "transient",
        "transient_step",
        "ladder stages",
        "One exact companion step from rest (timestep 1/4): lowering, assembly, solve and replay.",
    );
    for (kind, stiffness, policy) in STIFF_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 8),
            |n, _| rc_ladder(n as usize, stiffness, policy.clone()),
            |c| {
                Shaped::new(
                    checked(c.transient_step(q(1, 4), &TransientHistory::default())),
                    |report| {
                        let report = report.as_ref().ok()?;
                        let (nodes, depth, bits) = value_shape(&report.solution.candidate);
                        Some(Shape { nodes, depth, bits })
                    },
                )
            },
        );
    }
    atlas.operation(
        "transient",
        "diode_transient_run",
        "accepted steps",
        "Fixed-step mixed reactive/diode transient (timestep 1/4): Newton to 10^-8 at every step; a diode-clamped RC ladder.",
    );
    for stages in [1usize, 4] {
        atlas.sweep(
            &format!("{stages}-stage diode RC ladder"),
            &powers_of_two(0, 7),
            |n, _| {
                let mut c = rc_ladder(stages, Stiffness::NonStiff, TransientPolicy::Trapezoidal);
                let last = net(format!("n{}", stages - 1));
                c.instances.push(CircuitInstance {
                    id: CircuitInstanceId::new("Dclamp").unwrap(),
                    component: ComponentId::new("Dclamp").unwrap(),
                    part: None,
                    model: DeviceModelId::new("D").unwrap(),
                    pins: vec![
                        PinBinding {
                            pin: PinRef::new("+").unwrap(),
                            net: last,
                        },
                        PinBinding {
                            pin: PinRef::new("-").unwrap(),
                            net: net("GND"),
                        },
                    ],
                    parameters: Vec::new(),
                });
                c.source_stimuli.push(SourceStimulus {
                    component: ComponentId::new("V1").unwrap(),
                    waveform: SourceWaveform::Step {
                        initial: r(0),
                        final_value: r(1),
                        at: q(1, 2),
                    },
                });
                (c, fixed_policy(n as usize, q(1, 4)))
            },
            |(c, run)| {
                let tolerance = q(1, 100_000_000);
                checked(c.diode_transient_run(
                    run,
                    &DiodeNewtonPolicy {
                        maximum_iterations: 32,
                        voltage_tolerance: tolerance.clone(),
                        current_tolerance: tolerance,
                        damping: r(1),
                    },
                    TransientHistory::default(),
                    BTreeMap::new(),
                ))
                .and_then(|report| match report.status {
                    TransientRunStatus::Complete => Ok(report.samples.len()),
                    status => Err(format!("{status:?}")),
                })
            },
        );
    }
}

fn ac_ops(atlas: &mut Atlas) {
    let excitation = [AcExcitation::new(
        ComponentId::new("V1").unwrap(),
        Phasor::real(r(1)),
    )];
    atlas.operation(
        "ac",
        "ac_sweep",
        "frequency count",
        "Exact AC sweep of an 8-stage ladder over a growing list of angular frequencies k/8.",
    );
    for (kind, inductive) in [("RC ladder", false), ("RLC ladder", true)] {
        let excitation = excitation.clone();
        atlas.sweep(
            kind,
            &powers_of_two(0, 8),
            |n, _| (ac_ladder(8, inductive), n as i64),
            move |(c, count)| {
                checked(c.ac_sweep((1..=*count).map(|k| q(k, 8)), &excitation))
                    .map(|r| r.points.len())
            },
        );
    }
    atlas.operation(
        "ac",
        "ac_mna_at + solve_exact",
        "ladder stages",
        "Exact complex MNA at angular frequency 1, solved and replayed, on ladders of growing size.",
    );
    for (kind, inductive) in [("RC ladder", false), ("RLC ladder", true)] {
        let excitation = excitation.clone();
        atlas.sweep(
            kind,
            &powers_of_two(0, 7),
            |n, _| ac_ladder(n as usize, inductive),
            move |c| {
                let report = checked(c.ac_mna_at(r(1), &excitation))
                    .and_then(|system| checked(system.solve_exact()));
                Shaped::new(report, |report| {
                    let report = report.as_ref().ok()?;
                    let (nodes, depth, bits) = value_shape(
                        report
                            .candidate
                            .iter()
                            .flat_map(|p| [&p.real, &p.imaginary]),
                    );
                    Some(Shape {
                        nodes: nodes / 2,
                        depth,
                        bits,
                    })
                })
            },
        );
    }
}

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn pad(name: &str, x: i64) -> LandPatternPad {
    LandPatternPad {
        id: PadId::new(name).unwrap(),
        center: point(x, 0),
        rotation_degrees: r(0),
        copper_layers: vec![TraceLayer(0)],
        shape: PadShape::Circle { diameter: r(1) },
        drill: None,
        plating: Plating::Unspecified,
        solder_mask_margin: None,
        paste_margin: None,
    }
}

fn land_pattern(id: &str, pads: Vec<LandPatternPad>, pins: &[&str]) -> LandPattern {
    LandPattern {
        id: LandPatternId::new(id).unwrap(),
        pin_map: pins
            .iter()
            .zip(&pads)
            .map(|(pin, pad)| PadPinMap {
                pin: PinRef::new(*pin).unwrap(),
                pad: pad.id.clone(),
            })
            .collect(),
        pads,
        graphics: Vec::new(),
        body: None,
        models: Vec::new(),
    }
}

/// A resistor-ladder board of `n` components on a grid of 4-unit cells.
/// `stacked` places every component at the same origin, so placement must
/// resolve every collision.
#[derive(Clone)]
struct Board {
    circuit: Circuit,
    layout: PcbLayout,
}

fn ladder_board(n: usize, stacked: bool) -> Board {
    let circuit = resistor_network(
        "ladder",
        n.div_ceil(2).max(1),
        Scalar::Integer,
        &mut Rng::new(3),
    );
    let count = circuit.instances.len();
    let columns = (count as f64).sqrt().ceil() as i64;
    let rows = (count as i64 + columns - 1) / columns;
    let mut layout = PcbLayout::new(
        BoardId::new("atlas").unwrap(),
        BoardOutline::rectangle(r(columns * 4 + 4), r(rows * 4 + 4)).unwrap(),
        PcbStackup::single_layer(r(1), Some("copper".into())),
    );
    layout.land_patterns.push(land_pattern(
        "two-pad",
        vec![pad("1", -1), pad("2", 1)],
        &["+", "-"],
    ));
    for (i, instance) in circuit.instances.iter().enumerate() {
        let (x, y) = if stacked {
            (4, 4)
        } else {
            (i as i64 % columns * 4 + 4, i as i64 / columns * 4 + 4)
        };
        layout.placements.push(PcbPlacement {
            instance: instance.id.clone(),
            land_pattern: LandPatternId::new("two-pad").unwrap(),
            position: point(x, y),
            rotation_degrees: r(0),
            side: BoardSide::Front,
        });
    }
    Board { circuit, layout }
}

/// Two-terminal nets to route: `parallel` rows on one layer, or clusters of
/// two crossing nets on two layers.
fn routing_board(
    nets: usize,
    crossing: bool,
    pitch_divisor: i64,
) -> (Board, NegotiatedRoutePolicy) {
    let model = DeviceModelId::new("terminal").unwrap();
    let mut circuit = Circuit::new(
        CircuitId::new("routing").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Custom("terminal".into()),
        pins: vec![pin("1")],
        parameters: Vec::new(),
    });
    let mut pairs = Vec::new();
    let (width, height) = if crossing {
        let clusters = nets.div_ceil(2) as i64;
        for c in 0..clusters {
            let x = 1 + c * 10;
            pairs.push((point(x + 1, 5), point(x + 7, 5)));
            pairs.push((point(x + 4, 2), point(x + 4, 8)));
        }
        pairs.truncate(nets.max(1));
        (clusters * 10 + 2, 10)
    } else {
        for i in 0..nets.max(1) as i64 {
            pairs.push((point(3, 3 + 2 * i), point(21, 3 + 2 * i)));
        }
        (24, 2 * nets.max(1) as i64 + 4)
    };
    let mut placements = Vec::new();
    for (i, (a, b)) in pairs.iter().enumerate() {
        let id = net(format!("N{i}"));
        circuit = circuit.with_net(Net {
            id: id.clone(),
            is_ground: false,
        });
        for (suffix, position) in [("A", a), ("B", b)] {
            let name = format!("T{i}{suffix}");
            circuit = circuit.with_instance(CircuitInstance {
                id: CircuitInstanceId::new(name.clone()).unwrap(),
                component: ComponentId::new(name.clone()).unwrap(),
                part: None,
                model: model.clone(),
                pins: vec![PinBinding {
                    pin: PinRef::new("1").unwrap(),
                    net: id.clone(),
                }],
                parameters: Vec::new(),
            });
            placements.push(PcbPlacement {
                instance: CircuitInstanceId::new(name).unwrap(),
                land_pattern: LandPatternId::new("one-pad").unwrap(),
                position: position.clone(),
                rotation_degrees: r(0),
                side: BoardSide::Front,
            });
        }
    }
    let stackup = if crossing {
        PcbStackup::two_layer(r(1), r(1), Some("copper".into()), Some("FR4".into()))
    } else {
        PcbStackup::single_layer(r(1), Some("copper".into()))
    };
    let mut layout = PcbLayout::new(
        BoardId::new("routing").unwrap(),
        BoardOutline::rectangle(r(width), r(height)).unwrap(),
        stackup,
    );
    layout
        .land_patterns
        .push(land_pattern("one-pad", vec![pad("1", 0)], &["1"]));
    layout.placements = placements;
    // Finer grids scale trace, clearance and via sizes with the pitch, so the
    // same nets are routed on a k-times finer lattice.
    let defaults = NegotiatedRoutePolicy::default();
    let scale = |value: Real| (value / r(pitch_divisor)).expect("nonzero divisor");
    let policy = NegotiatedRoutePolicy {
        grid_pitch: scale(defaults.grid_pitch.clone()),
        default_trace_width: scale(defaults.default_trace_width.clone()),
        default_clearance: scale(defaults.default_clearance.clone()),
        via_land_diameter: scale(defaults.via_land_diameter.clone()),
        via_drill_diameter: scale(defaults.via_drill_diameter.clone()),
        maximum_iterations: 16,
        maximum_expansions_per_connection: 2_000_000,
        ..defaults
    };
    (Board { circuit, layout }, policy)
}

fn route(board: &Board, policy: &NegotiatedRoutePolicy) -> Shaped<Checked<usize>> {
    let report = checked(
        board
            .layout
            .negotiated_autoroute(&board.circuit, policy.clone()),
    )
    .and_then(|report| match report.status {
        NegotiatedRouteStatus::Complete => Ok(report.work.expanded_states_total),
        status => Err(format!(
            "{status:?} after {} passes",
            report.work.iterations_executed
        )),
    });
    Shaped::new(report, |states| {
        states.as_ref().ok().map(|&states| Shape {
            nodes: states as u64,
            depth: 0,
            bits: 0,
        })
    })
}

fn layout_ops(atlas: &mut Atlas) {
    let counts = powers_of_two(1, 10);
    atlas.operation(
        "layout",
        "PcbLayout::validate",
        "component count",
        "Structural layout validation (identities, land patterns, pin maps, placements) against the circuit.",
    );
    for (kind, stacked) in [("grid placement", false), ("stacked at origin", true)] {
        atlas.sweep(
            kind,
            &counts,
            |n, _| ladder_board(n as usize, stacked),
            |b| b.layout.validate(&b.circuit).is_valid(),
        );
    }
    atlas.operation(
        "layout",
        "solve_placement",
        "component count",
        "Exact collision-free row-major placement repair (default policy); stacked boards must move every component but one.",
    );
    for (kind, stacked) in [
        ("legal grid (no moves)", false),
        ("stacked at origin", true),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(1, 8),
            |n, _| ladder_board(n as usize, stacked),
            |b| {
                let report = b
                    .layout
                    .solve_placement(&b.circuit, &PlacementSolvePolicy::default());
                if report.is_solved() {
                    Ok(report.moves.len())
                } else {
                    Err(format!("{:?}", report.issues.first()))
                }
            },
        );
    }
    atlas.operation(
        "routing",
        "RoutingProblemReport::from_layout",
        "net count",
        "Lowering a placed layout to a hyperpath routing problem (terminals, obstacles, rules).",
    );
    for (kind, crossing) in [
        ("parallel bus, 1 layer", false),
        ("crossing pairs, 2 layers", true),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 8),
            |n, _| routing_board(n as usize, crossing, 1).0,
            |b| checked(RoutingProblemReport::from_layout(&b.circuit, &b.layout)).map(|_| ()),
        );
    }
    atlas.operation(
        "routing",
        "negotiated_autoroute (nets)",
        "net count",
        "Negotiated-congestion grid autorouting at unit pitch (at most 16 rip-up passes); nodes count expanded search states.",
    );
    for (kind, crossing) in [
        ("parallel bus, 1 layer", false),
        ("crossing pairs, 2 layers", true),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 7),
            |n, _| routing_board(n as usize, crossing, 1),
            |(b, policy)| route(b, policy),
        );
    }
    atlas.operation(
        "routing",
        "negotiated_autoroute (resolution)",
        "grid cells per unit",
        "Four parallel nets, or one crossing pair, routed with grid pitch, trace, clearance and via sizes scaled by 1/k: grid states grow with k^2.",
    );
    for (kind, crossing, nets) in [
        ("parallel bus (4 nets), 1 layer", false, 4),
        ("crossing pair (2 nets), 2 layers", true, 2),
    ] {
        atlas.sweep(
            kind,
            &ladder(1, 16, 8),
            |k, _| routing_board(nets, crossing, k as i64),
            |(b, policy)| route(b, policy),
        );
    }
    atlas.operation(
        "exchange",
        "export_kicad",
        "component count",
        "Standalone .kicad_pcb text (plus rules and project companions) for a placed ladder board.",
    );
    atlas.sweep(
        "grid placement",
        &counts,
        |n, _| ladder_board(n as usize, false),
        |b| {
            checked(
                b.layout
                    .export_kicad(&b.circuit, KiCadExportOptions::default()),
            )
            .map(|report| report.board.len())
        },
    );
    atlas.operation(
        "exchange",
        "KiCadImportReport::from_str",
        "component count",
        "Parse an exported .kicad_pcb back into an editable circuit and layout.",
    );
    atlas.sweep(
        "grid placement",
        &counts,
        |n, _| {
            let b = ladder_board(n as usize, false);
            b.layout
                .export_kicad(&b.circuit, KiCadExportOptions::default())
                .expect("export")
                .board
        },
        |text| {
            Shaped::new(
                checked(KiCadImportReport::from_str(
                    text,
                    KiCadImportOptions::new(
                        CircuitId::new("imported").unwrap(),
                        BoardId::new("imported").unwrap(),
                        r(1),
                    ),
                )),
                |report| {
                    report.as_ref().ok().map(|report| Shape {
                        nodes: report.layout.placements.len() as u64,
                        depth: 0,
                        bits: 0,
                    })
                },
            )
        },
    );
    #[cfg(feature = "geometry")]
    {
        use hypercircuit::{MaterializationContext, MaterializationOptions};
        atlas.operation(
            "geometry",
            "PcbLayout::materialize",
            "component count",
            "Exact substrate, pad copper and drill geometry through csgrs; with layer images, copper is also boolean-unioned per layer.",
        );
        for (kind, aggregate) in [("features only", false), ("with layer images", true)] {
            atlas.sweep(
                kind,
                &powers_of_two(1, 8),
                |n, _| ladder_board(n as usize, false),
                |b| {
                    let report = checked(b.layout.materialize(
                        &b.circuit,
                        &MaterializationContext::STRICT,
                        MaterializationOptions {
                            aggregate_layer_images: aggregate,
                            circular_segments: 16,
                            ..MaterializationOptions::default()
                        },
                    ));
                    Shaped::new(report, |report| {
                        report.as_ref().ok().map(|report| Shape {
                            nodes: report.copper_features.len() as u64,
                            depth: 0,
                            bits: 0,
                        })
                    })
                },
            );
        }
    }
}

fn main() {
    let mut atlas = Atlas::from_args("hypercircuit");
    netlist_ops(&mut atlas);
    mna_ops(&mut atlas);
    nonlinear_ops(&mut atlas);
    transient_ops(&mut atlas);
    ac_ops(&mut atlas);
    layout_ops(&mut atlas);
    atlas.finish();
}
