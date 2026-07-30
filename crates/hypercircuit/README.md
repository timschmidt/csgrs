# HyperCircuit

HyperCircuit is the semantic circuit and PCB layer of the Hyper workspace. It
keeps electrical identity, hierarchy, interfaces, simulation intent,
schematics, layout intent, fabrication evidence, and release policy connected
without treating a solver matrix, drawing, or mesh as the source of truth.

Core circuit values use `hyperreal::Real`. Numerical solvers and external tools
are proposal adapters: accepted circuit results retain exact residual replay or
an explicit uncertainty boundary.

The crate ranges from a small exact-aware Modified Nodal Analysis (MNA) model
to optional PCB authoring, routing, KiCad exchange, manufacturing release, and
HyperDRC handoff. Features let applications select only the layers they need.

## What HyperCircuit is for

A circuit design has several related representations:

```text
parts + models + nets + hierarchy + intent
                    │
          ┌─────────┼───────────┐
          ▼         ▼           ▼
     simulation  schematic   PCB layout
          │                     │
          ▼                     ▼
    exact replay       routing / geometry / DRC
                                │
                                ▼
                  fabrication + assembly release
```

HyperCircuit keeps stable typed identities across those branches. A net remains
the same semantic net when lowered into MNA unknowns, schematic wires,
Hyperpath routing carriers, CSGRS copper geometry, HyperDRC findings, Gerber
attributes, and assembly outputs.

Ownership is deliberately separated:

- HyperCircuit owns circuit, schematic, PCB, and release semantics.
- [Hyperpath](https://github.com/timschmidt/hyperpath) owns routed-path geometry and exact path
  predicates.
- [CSGRS](https://github.com/timschmidt/csgrs) composes profiles and solids for materialization.
- [HyperDRC](https://github.com/timschmidt/hyperdrc) owns readiness policy and findings.
- [Hyperphysics](https://github.com/timschmidt/hyperphysics) owns physical property models.
- [Hypersolve](https://github.com/timschmidt/hypersolve) accepts coupled residual systems.

## Primary types

- `Circuit` is the retained electrical model: nets, buses, ports, rails,
  device models, instances, manual stamps, stimuli, module parameters, and
  child circuits.
- `CircuitId`, `NetId`, `ComponentId`, `DeviceModelId`, `CircuitInstanceId`,
  `PortId`, and the other ID wrappers prevent accidental cross-domain identity
  mixing.
- `DeviceModel` and `DeviceModelKind` describe supported electrical behavior;
  `CircuitInstance` binds model pins to nets and supplies exact parameters.
- `LinearStamp`, `MnaProblem`, `LinearMnaSystem`, and `MnaUnknown` are the
  executable linear MNA layer.
- `ResidualReplayReport`, `LinearSolveReport`, and nonlinear/AC/transient
  reports distinguish a proposal from its checked result.
- `CircuitLibrary`, `SubcircuitInstance`, and `CircuitModuleParameter` model
  reusable hierarchy and parameter forwarding.
- `SchematicLayout` and its symbol/wire/sheet types retain presentation
  without duplicating logical connectivity.
- With `layout`, `PcbLayout` and its land-pattern, placement, route, via, zone,
  stackup, rule, tuning, and test-intent types retain board semantics.
- With `interchange`, `SemanticDocument` is the versioned deterministic
  persistence boundary.
- With `drc`, `ReleaseReport` and `HyperDrcReadinessReport` connect design
  semantics to release blockers and evidence.

## Installation

The circuit and simulation core has no default features:

```toml
[dependencies]
hypercircuit = "0.3.0"
```

Enable PCB authoring and semantic interchange as needed:

```toml
[dependencies]
hypercircuit = { version = "0.3.0", features = ["layout", "interchange"] }
```

## Quick start

This one-node conductance problem lowers to exact MNA, solves it, and replays
the residual:

<!-- quickstart:start -->
```rust
use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitResult, ComponentId, LinearStamp, Net, NetId, Real,
    TransientPolicy,
};

fn main() -> CircuitResult<()> {
    let output = NetId::new("out")?;
    let circuit = Circuit::new(
        CircuitId::new("conductance")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_stamp(LinearStamp::Conductance {
        component: ComponentId::new("g1")?,
        part: None,
        pos: Some(output),
        neg: None,
        conductance: Real::from(2),
    });

    let system = circuit.linear_mna_system()?;
    let solution = system.solve_exact()?;
    assert!(solution.replay.accepted);
    assert_eq!(solution.candidate, vec![Real::zero()]);
    Ok(())
}
```
<!-- quickstart:end -->

Run the repository copy with:

```sh
cargo run --example basic
```

The example uses an explicit stamp to make the MNA boundary visible. Most
applications author `DeviceModel` and `CircuitInstance` values, then call
`linear_mna_from_devices`.

## Useful API

The generated Rust documentation contains full signatures and report fields.
The groups below cover the useful public surface.

### Circuit identity, models, and validation

- core model: `Circuit`, `Net`, `Bus`, `BusSlice`, `CircuitPort`,
  `RailIntent`, `CircuitInstance`, `DeviceModel`, `DeviceModelKind`,
  `DevicePin`, `PinBinding`, `CircuitParameter`;
- identity: the types in `identity`, including circuit, component, net, pin,
  port, layout, package, event, route, via, zone, and tuning IDs;
- policy: `TransientPolicy`, `AdapterKind`, `PortDirection`,
  `PinElectricalKind`, `PortSignalType`, `RailKind`, `MosfetPolarity`;
- validation: `Circuit::validate`, `CircuitValidationReport`,
  `CircuitValidationIssue`, and `CircuitCertificationReport`;
- intent: `DesignIntent`, `NetIntent`, `FunctionalRole`,
  `FunctionalBinding`, `PartSelectionIntent`, `ResolvedPartEvidence`,
  `SourceSpan`, and their validation reports.

Builder methods on `Circuit` add nets, buses, ports, rails, models, instances,
stamps, stimuli, module parameters, and subcircuits while preserving stable
identity.

### Linear MNA and exact replay

- `Circuit::linear_mna_system` lowers authored `LinearStamp` values.
- `Circuit::linear_mna_from_devices` lowers supported device instances.
- `LinearMnaSystem::solve_exact` performs certified-pivot dense elimination
  and replays its own candidate.
- `LinearMnaSystem::replay_candidate` evaluates exact `A*x-b` residuals for an
  external proposal.
- `MnaProblem`, `LinearStamp`, `MnaUnknown`, `LinearSolveReport`, and
  `ResidualReplayReport` expose the equation and evidence model.

Supported linear stamp families include conductance, current and voltage
sources, voltage-controlled current sources, and transient capacitor/inductor
companions. Duplicate unknowns and uncertified pivots are errors.

### Transient, events, and behavior

- sources: `SourceStimulus`, `SourceWaveform`, `SourceWaveformPoint`;
- one-step simulation: `transient_step`, `transient_step_at`,
  `TransientStepReport`, `ReactiveState`;
- bounded series: `transient_run`, `TransientRunPolicy`,
  `TransientAdaptation`, `TransientRunReport`, `TransientRunStatus`;
- stateful execution: `TransientSession`, `TransientSessionStep`,
  `TransientSessionAuditReport`;
- exact breakpoints: `ExactBreakpointProvider`, `ExactBreakpointSchedule`,
  `earliest_exact_breakpoint_after`;
- events: `CircuitEventAgenda`, `CircuitEventRequest`, `CircuitEvent`,
  phase/kind/target/cause types, lifecycle records, counters, replay, and
  deterministic fingerprints;
- behavior: `BehaviorRuntime`, `CircuitEventHandler`, `BehaviorContext`,
  `BehaviorQueue`, and `SelectiveEventMailbox`;
- asynchronous cooperative behavior behind `behavior-async`:
  `AsyncBehaviorRuntime` and `AsyncCircuitBehavior`;
- reproducible randomness: `StochasticStream`, `StochasticSample`, and
  `RandomDrawId`.

Constant, step, piecewise-linear, periodic pulse, damped-sine, and dual-edge
exponential stimuli keep exact timing parameters. Adaptive linear runs compare
replayed full and half steps under caller-authored bounds.

### Nonlinear DC and AC

- piecewise-linear: `PiecewiseLinearDevice`, `PiecewiseLinearSegment`,
  `solve_piecewise_linear`, and its bounded active-region report;
- diodes: `ShockleyDiode`, `DiodeNewtonPolicy`,
  `solve_shockley_diode_newton`, diode transient step/run reports;
- MOSFETs: `SquareLawMosfet`, `MosfetRegion`, `MosfetNewtonPolicy`,
  `solve_square_law_mosfet_newton`, `Circuit::solve_mosfet_dc`;
- AC: `AcMnaSystem`, `AcStamp`, `Phasor`, `AcExcitation`,
  `AcOperatingPoint`, `AcSolveReport`, and sweep reports.

Primitive-float exponential/Newton coefficients are retained as proposal
evidence. Acceptance replays the true device law against explicit voltage,
current, and residual policies. The current MOSFET model is the body-tied-source
level-one Shichman–Hodges law; dynamic capacitance, body effect, subthreshold,
and reverse-channel behavior are not implied.

### Hierarchy, interfaces, and packages

- hierarchy: `CircuitLibrary`, `SubcircuitInstance`,
  `SubcircuitPortBinding`, `flatten`, `flatten_with_scopes`,
  `CircuitFlatteningReport`;
- parameters: `CircuitModuleParameter`,
  `CircuitModuleParameterTarget`, and overrides;
- typed bundles: `SignalBundle`, `SignalBundleMember`, `Modport`,
  `SignalBundleEndpoint`, `SignalBundleLibrary`;
- design modules: `DesignModule`, `DesignModuleInstance`, `CheckedProject`;
- package resolution: `CircuitPackageCatalog`, `PackageRequirement`,
  `CircuitPackageLock`, `PackageDigest`, `CircuitPackageStore`;
- portable library artifacts: `CircuitLibraryArtifact`,
  `PartLibraryArtifact`, and portable part definitions.

Hierarchy validation rejects recursion and missing or incompatible bindings.
Flattening produces stable path-qualified identities and deterministic scope
maps.

### ERC and semantic authoring

- ERC: `electrical_rule_check`, `ErcRuleDeck`, `ErcReport`,
  `ConfiguredErcReport`, `ErcFinding`, `ErcRuleId`, and severity types;
- high-level authoring with `layout`: `Design`, `CheckedDesign`, `Part`,
  `PartDefinition`, `PartInstance`, typed handles, authoring traces, and source
  maps;
- reusable symbols and footprints: `PartSymbolUnit`, `SymbolUnitPlacement`,
  `Footprint`, `LandPattern`, and pin-to-pad maps.

ERC covers driven-net conflicts, undriven inputs, power-source evidence,
no-connect misuse, ground/signal role mismatches, and copper-trace handoff
issues. It is electrical-intent review, not analog stability proof.

### Schematics and KiCad schematic exchange

- drawing model: `SchematicLayout`, `SchematicSymbolDefinition`,
  `SchematicSymbol`, `SchematicWire`, labels, sheets, sheet ports/links,
  graphics, presentation, and validation reports;
- generation: `auto_schematic`, `SchematicAutoLayoutPolicy`,
  `SchematicAutoLayoutReport`;
- rendering: schematic SVG options, reports, and numeric projections;
- KiCad: schematic import/export reports, book import/export, numeric
  projection/import evidence, and typed omissions.

Schematic objects point back to circuit identities. They do not create a
second connectivity model.

### PCB layout, placement, and routing

Available with `layout`:

- board model: `PcbLayout`, `BoardOutline`, `BoardContour`,
  `BoardBoundaryGeometry`, `PcbStackup`, and stackup-layer types;
- libraries and placement: `LandPattern`, pads, graphics, 3D model references,
  `PcbPlacement`, placement constraints, `PlacementSolvePolicy`, and reports;
- routing: `PcbRoute`, `PcbRouteSegment`, `PcbVia`, `ViaStyle`, `NetClass`,
  `DifferentialPair`, `RouteConstraintRegion`, `RouteRuleRegion`,
  `EscapePolicy`;
- zones and finishing: `CopperZone`, fill/island/stitching policy and evidence;
- tuning: `LengthTuningPattern`, `PhaseTuningGroup`, synthesis/realization
  policies and reports;
- hierarchy: `LayoutModule`, `PlacementGroup`, `LayoutAssembly`, and composed
  layout reports;
- panels and tests: `PanelDefinition`, rails, coupons, fiducials, tooling,
  separation features, `DesignForTestIntent`, test access, boundary scan,
  programming, and power-domain requirements.

`RoutingProblemReport::from_layout` lowers placed-pad terminals, path rules,
keepouts, fixed copper, and aliases into Hyperpath. `RoutingSolution` imports
accepted Hyperpath paths back into semantic route/via identities.

`PcbLayout::negotiated_autoroute` and
`adaptive_negotiated_autoroute` provide bounded deterministic proposal
engines. Their policies and reports retain grid topology, refinements, passes,
expansions, conflicts, failures, differential-pair evidence, via styles,
regional rules, and exact quality metrics. `iteration_svg` and `replay_html`
render that retained evidence without becoming another routing IR.

The tsCircuit adapter exports and imports `SimpleRouteJson` with typed
`TscircuitRoutingOmission` values for information the finite protocol cannot
carry.

### Geometry, fabrication, DRC, and release

- `geometry`: exact-coordinate layout/material carriers lowered through CSGRS;
- `materialize`: copper/process features, layer images, drill hits, projection
  evidence, and 3D assembly/glTF reports;
- `fabrication`: Gerber/X3, Gerber Job, Excellon, IPC-D-356, manifests,
  contour policy, and CAM round-trip evidence;
- `assembly`: deterministic BOM, pick-and-place, DNP, variants, and CSV
  round-trip audit;
- `drc`: `HyperDrcHandoff`, material-property evidence,
  `HyperDrcReadinessReport`, and readiness policy;
- `workflow`: `ReleaseOptions`, `ReleaseReport`, `ReleaseBlocker`;
- `manufacturing_release`: deterministic directory/ZIP bundles, schema,
  signatures, verification, and difference reports.

See [manufacturing release](docs/manufacturing-release.md) for the CLI and
bundle contract. Fabrication projection is explicit; exact curve and route
intent is not silently relabeled as exact Gerber geometry.

### Interchange, editing, and external tools

Available with `interchange` unless noted:

- `SemanticDocument`, schema constants, migration reports, deterministic JSON;
- `ProjectManifest`, named design providers, project/material metadata;
- `DesignEdit`, `DesignEditBatch`, `DesignHistory`, revisions, undo/redo,
  optimistic rebasing, merge conflicts, and replay reports;
- KiCad PCB import/export, numeric evidence, companion project/rule files, and
  typed omissions;
- KiCad symbol/footprint library import;
- `IntelligentPcbExchangeAdapter`, subprocess adapter, IPC-2581 inspection,
  manifests, packages, and limits;
- release-artifact catalogs and portable paths;
- `LcedaProExportReport` and baseline-relative import with `lceda`.

The `hypercircuit` binary, enabled by `drc,interchange`, loads a versioned
`hypercircuit.toml` or a direct `SemanticDocument` and exposes:

```text
hypercircuit check
hypercircuit ir
hypercircuit snapshot
hypercircuit bom
hypercircuit export-kicad
hypercircuit export-svg
hypercircuit release build
hypercircuit release verify
```

Run `cargo run --features drc,interchange --bin hypercircuit -- --help` for
the generated command contract.

`LegacyCsgrsElectronicsImport`, behind `geometry`, is migration-only support
for old versioned CSGRS electrical claims. It never infers connectivity from a
mesh and is scheduled for removal in HyperCircuit 0.4.0.

### Multiphysics handoffs

- `PhysicalElectricalPort`, `ElectromechanicalPort`, and `ThermalPort`;
- `CoupledResidualBlock` and `to_hypersolve_problem`;
- `ElectrothermalRcReport`;
- `CircuitAdapterReport` and `ElectrothermalTraceFixture`.

These are boundary records. HyperCircuit does not embed a general field solver.

## Features

| Feature | Adds |
|---|---|
| default | circuit model, exact dense MNA/replay, transient, nonlinear DC, AC, events, hierarchy, ERC, schematic, package resolution |
| `behavior-async` | cooperative asynchronous behavior callbacks |
| `dispatch-trace` | Hyperreal computation-path evidence |
| `layout` | PCB authoring, placement, routing, tuning, panel, assembly, Hyperpath/Hypercurve integration |
| `geometry` | CSGRS materialization, fabrication geometry, 3D scene reports, legacy CSGRS migration |
| `drc` | HyperDRC and Hyperphysics property handoff; implies `geometry` |
| `interchange` | semantic JSON, project manifests, edit history, KiCad/package/release exchange |
| `lceda` | EasyEDA/LCEDA Pro adapter; implies `interchange` |

Features are cumulative where shown in `Cargo.toml`: `geometry` includes
`layout`, `drc` includes `geometry`, and `lceda` includes `interchange`.

## Guarantees and boundaries

- Stable typed identities connect circuit, schematic, PCB, fabrication,
  assembly, and report records.
- Exact MNA rows, unknown order, parameters, source times, domains, and
  residuals remain structured and replayable.
- A numerical tolerance or finite projection is never presented as an exact
  proof. Adapter omissions and uncertainty remain typed.
- The built-in dense solver is intended for small systems and certification
  paths. Sparse matrices, general DAE solving, dynamic MOSFET effects,
  adaptive nonlinear multistep integration, and field solvers are external
  adapter territory.
- Layout geometry belongs to Hyperpath/Hypercurve/CSGRS; readiness policy
  belongs to HyperDRC. HyperCircuit retains semantic source identity through
  those calls.
- KiCad, Gerber, IPC-D-356, SVG, glTF, EasyEDA, and tsCircuit cannot represent
  every retained concept. Import/export reports enumerate numeric projection
  and semantic omissions.
- Manufacturing release is deterministic evidence packaging, not fabrication
  certification.
- Performance measurements, rejected experiments, and machine-neutral routing
  work units are maintained in [PERFORMANCE.md](PERFORMANCE.md), not embedded
  as release promises.

The [capability matrix](CAPABILITY_MATRIX.md) records detailed tsCircuit/via-rs
equivalence and acceptance gates; the README describes only supported public
behavior.

## Examples

Representative runnable examples include:

- `basic`, `analytic_sources`, `transient_run`, `diode_transient`,
  `mosfet_dc`, `ac_sweep`, `nonlinear_ac`, and `mixed_signal_session`;
- `parameterized_module`, `signal_bundles`, `hierarchical_schematic`,
  `auto_schematic`, and `kicad_schematic`;
- `declarative_board`, `layout_module`, `advanced_routing`,
  `phase_tuning`, `curved_fabrication`, and `review_bundle`;
- `kicad_roundtrip`, `kicad_library_import`, `semantic_edit`,
  `lceda_pro_export`, and `tscircuit_router_handoff`;
- the license-clean EasyDuino fixture generators and release pipelines.

Cargo reports a missing required feature when an example needs an optional
layer. For example:

```sh
cargo run --features layout --example advanced_routing
cargo run --features interchange --example semantic_edit -- --help
cargo run --features drc,interchange --example easyduino_nano
```

## References

Circuit analysis and simulation:

- C.-W. Ho, A. Ruehli, and P. Brennan,
  [“The Modified Nodal Approach to Network Analysis”](https://doi.org/10.1109/TCS.1975.1084079),
  *IEEE Transactions on Circuits and Systems*, 1975.
- Laurence W. Nagel,
  [*SPICE2: A Computer Program to Simulate Semiconductor Circuits*](https://www2.eecs.berkeley.edu/Pubs/TechRpts/1975/9602.html),
  UCB/ERL M520, 1975.
- D. A. Calahan,
  [“Computer-Aided Network Design—Modified Nodal Analysis”](https://doi.org/10.1109/TCT.1967.1082701),
  *IEEE Transactions on Circuit Theory*, 1967.
- H. Shichman and D. A. Hodges,
  [“Modeling and Simulation of Insulated-Gate Field-Effect Transistor Switching Circuits”](https://doi.org/10.1109/JSSC.1968.1049902),
  *IEEE Journal of Solid-State Circuits*, 1968.
- Lawrence Livermore National Laboratory,
  [SUNDIALS IDA](https://computing.llnl.gov/projects/sundials/ida), the
  DAE/BDF adapter family named by `TransientPolicy::IdaDaeAdapter`.

Geometry, routing, and coupled systems:

- Chee K. Yap,
  [“Towards Exact Geometric Computation”](https://doi.org/10.1016/0925-7721(95)00040-2),
  *Computational Geometry* 7, 1997.
- Larry McMurchie and Carl Ebeling,
  [“PathFinder: A Negotiation-Based Performance-Driven Router for FPGAs”](https://www.cs.cmu.edu/~440/Pathfinder.pdf),
  FPGA 1995. HyperCircuit adopts the bounded negotiation pattern, not the FPGA
  resource model.
- Sebastian Cortes Garcia, Herbert De Gersem, and Sebastian Schöps,
  [“A Structural Analysis of Field/Circuit Coupled Problems Based on a Generalised Circuit Element”](https://doi.org/10.1007/s11075-019-00686-x),
  *Numerical Algorithms*, 2020.

Interchange and manufacturing:

- KiCad,
  [official file-format documentation](https://dev-docs.kicad.org/en/file-formats/).
- Ucamco,
  [Gerber Layer and Job Format specifications](https://www.ucamco.com/en/gerber/downloads).
- IPC,
  [IPC-D-356B bare-board electrical test format](https://shop.electronics.org/ipc-d-356/ipc-d-356-standard-only).
- IPC,
  [IPC-2581C manufacturing-description transfer standard](https://shop.ipc.org/ipc-2581/ipc-2581-standard-only).
- Khronos Group,
  [glTF 2.0 specification](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html).
- tsCircuit,
  [SimpleRouteJson protocol](https://github.com/tscircuit/tscircuit-autorouter#input-format-simpleroutejson).

## Acknowledgements

HyperCircuit is built on the Hyperreal, Hypersolve, Hyperlattice, Hyperlimit,
Hypercurve, Hyperpath, CSGRS, HyperDRC, and Hyperphysics crates. Its exchange
layers also depend on the work of the KiCad, Ucamco, IPC, Khronos, tsCircuit,
EasyEDA/LCEDA, serde, and Rust communities.

The EasyDuino regression fixtures preserve their upstream licensing and
provenance in `tests/fixtures/easyduino/UPSTREAM_LICENSE.txt`. Thanks to the
authors of the circuit-analysis, routing, exact-geometry, and coupled-system
references above for making the implementation boundaries reviewable.

## Development

```sh
cargo fmt --all -- --check
cargo test --all-features --all-targets
cargo clippy --all-features --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
cargo check --all-features --benches
```

## License

HyperCircuit is licensed under Apache-2.0 as declared in `Cargo.toml`.
