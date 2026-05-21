<h1>
  hypercircuit
</h1>

`hypercircuit` is the exact-aware circuit carrier crate for the Hyper ecosystem. It
records circuit identity, device models, Modified Nodal Analysis (MNA) stamps, residual
replay, nonlinear-device policy, and electrothermal coupling reports over
`hyperreal::Real` values.

The crate is not a SPICE replacement. It is the boundary where authored circuit truth is
kept separate from numeric solver proposals, so downstream engines can be fast without
becoming the only record of whether a candidate state satisfies the model.

## Hyper Ecosystem

`hypercircuit` sits between electrical domain data and exact-aware solvers.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact scalar values for
  stamps, parameters, and residual replay.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): vector and tensor carriers
  used by sibling physical and field crates.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate and policy
  layer used by geometry/physics consumers.
- [hypertri](https://github.com/timschmidt/hypertri), [hypercurve](https://github.com/timschmidt/hypercurve),
  [hypermesh](https://github.com/timschmidt/hypermesh), and
  [hypervoxel](https://github.com/timschmidt/hypervoxel): geometry and sampled-field
  evidence for package, field, and fixture coupling.
- [hypersolve](https://github.com/timschmidt/hypersolve): future nonlinear residual,
  DAE, and candidate-certification surface.
- [hyperparts](https://github.com/timschmidt/hyperparts): part, terminal, package, and
  source-attributed electrical metadata.
- [hyperpath](https://github.com/timschmidt/hyperpath) and
  [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB trace/routing and release
  package evidence used by coupled fixtures.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): materials, thermal
  carriers, and physical coupling reports.
- [hyperpack](https://github.com/timschmidt/hyperpack): package and placement evidence
  for electromechanical assemblies.
- [hyperevolution](https://github.com/timschmidt/hyperevolution): proposal-search layer
  for circuit and coupled-design candidates.
- [hyperbrep](https://github.com/timschmidt/hyperbrep): exact boundary-representation
  geometry for future electromechanical fixtures.
- [hypersdf](https://github.com/timschmidt/hypersdf): implicit-field and clearance
  evidence for future electrical/physical coupling.

## Typical Circuit Problems

Circuit simulators usually optimize sparse floating-point iteration. That is appropriate
for large operating-point, AC, and transient runs, but it can hide whether a reported
state satisfies the authored equations or merely passed a tolerance. Mixed nonlinear,
switched, electrothermal, or field-coupled models add another failure mode: event policy,
device law, solver tolerance, and residual status often collapse into one opaque result.

`hypercircuit` treats solvers as proposal engines. Circuit topology, model parameters,
stamps, coupling ports, event policy, adapter precision, and exact residual replay stay
visible. A candidate can be accepted, rejected, or marked unknown without pretending the
numeric adapter was the source of truth.

## Main Types

- `Circuit`, `Net`, `CircuitInstance`, `DeviceModel`, `CircuitState`, `CircuitParameter`,
  and stable ID types describe circuit structure before solver lowering.
- `LinearStamp`, `MnaProblem`, `MnaUnknown`, and `LinearMnaSystem` represent the current
  exact linear MNA path.
- `ResidualReplayReport` records exact `A*x - b` replay status for proposed solutions.
- `NonlinearDeviceReport`, `PiecewiseLinearSegment`, `EventPolicy`, and `SwitchState`
  capture nonlinear and event-device setup before Newton or event lowering exists.
- `PhysicalElectricalPort`, `ThermalPort`, `CoupledResidualBlock`, and
  `ElectrothermalRcReport` describe circuit/physics coupling payloads.
- `CircuitAdapterReport` and `ElectrothermalTraceFixture` keep adapter kind, tolerance,
  and exact replay status explicit.
- `AdapterKind`, `CircuitCertificationReport`, `MnaProblem`, `CircuitParameter`, and
  `CircuitState` keep solver policy, provenance, and state separate from stamps.

## Precision Model

Circuit truth is represented with `hyperreal::Real`. Primitive floats should appear only
inside named numeric adapters, imported data, diagnostics, or external solver bridges.
The current linear path constructs exact dense MNA rows and replays exact residuals for
candidate vectors. For nonlinear and coupled systems, the crate stores exact parameters,
domains, piecewise-linear segments, slope facts, and event policy even when the eventual
solver is approximate.

Unknown replay is a valid result. It is preferable to accepting a candidate whose
precision boundary was not certified.

Numerical explosion is controlled by preserving topology, stamps, unknown ordering,
device-domain facts, and coupled residual blocks as structured records. The crate does
not eagerly expand every nonlinear device into a global expression tree or every
transient policy into a dense time system; adapters and future solvers must report the
boundary they cross.

## Performance Model

The present implementation favors clear carriers and exact replay over high-performance
sparse solving. Performance work is expected to come from:

- keeping IDs stable so sparse adapters can map rows and unknowns without rebuilding
  semantic context;
- separating stamp construction from residual replay;
- carrying model and event facts so future solvers can avoid unnecessary nonlinear work;
- using dense exact systems only for small fixtures and certification paths;
- making large sparse, transient, DAE, and field solvers explicit adapters with reports.
- lowering coupled residual blocks into `hypersolve` only at the replay boundary, so
  circuit ownership of ports and device evidence remains intact.

## Current Status

Implemented today:

- stable IDs and circuit/model/state carriers;
- conductance, source, controlled-source, ideal-voltage-source, and transient companion
  stamps;
- exact dense linear MNA system construction and residual replay;
- report-bearing nonlinear-device, switch, event-policy, and piecewise-linear setup;
- exact electrothermal RC coupling reports and trace-fixture handoff records.

Known limits: full diode/MOSFET laws, sparse matrix backends, Newton policy, transient
integration, DAE solving, and field/circuit coupling remain future adapter or solver
work.

## Installation

```toml
[dependencies]
hypercircuit = "0.2.0"
```

For sibling checkouts:

```toml
[dependencies]
hypercircuit = { path = "../hypercircuit" }
```

## Usage

Build circuit facts first, then lower or replay through explicit adapter surfaces:

```rust,ignore
use hypercircuit::{
    AdapterKind, Circuit, CircuitId, ComponentId, LinearStamp, Net, NetId,
    TransientPolicy,
};
use hyperreal::Real;

let ground = NetId::new("0")?;
let out = NetId::new("out")?;

let circuit = Circuit::new(
    CircuitId::new("divider")?,
    TransientPolicy::Static,
    AdapterKind::Dc,
)
.with_net(Net { id: ground.clone(), is_ground: true })
.with_net(Net { id: out.clone(), is_ground: false })
.with_stamp(LinearStamp::Conductance {
    component: ComponentId::new("r1")?,
    part: None,
    pos: Some(out.clone()),
    neg: None,
    conductance: Real::from(1),
});

let system = circuit.linear_mna_system()?;
let replay = system.replay_candidate(&[Real::zero()])?;
assert!(replay.accepted);
```

Other major surfaces follow the same pattern: `NonlinearDeviceReport` records device
law and event policy before Newton-style proposal engines exist, while
`ElectrothermalRcReport` and `CircuitAdapterReport` keep coupled physics and numeric
adapter status separate from circuit truth.

```rust,ignore
use hypercircuit::{
    CircuitParameter, ComponentId, ElectrothermalRcReport, NonlinearDeviceReport,
};
use hyperreal::Real;

let diode = NonlinearDeviceReport::diode(
    ComponentId::new("d1")?,
    vec![CircuitParameter {
        name: "is".into(),
        value: Real::from(1),
        unit: "A".into(),
        source: "fixture".into(),
    }],
);
assert_eq!(diode.domains.len(), 1);

let thermal = ElectrothermalRcReport::replay(
    ComponentId::new("r1")?,
    Real::from(100),
    Real::from(0),
    Real::from(25),
    Real::from(25),
    Real::from(2),
    "thermal-node-0",
)?;
assert_eq!(thermal.joule_heating, Real::from(400));
```

## References

- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry* 7.1-2
  (1997): 3-23.
- Ho, Chung-Wen, Albert E. Ruehli, and Pierce A. Brennan. "The Modified Nodal Approach
  to Network Analysis." *IEEE Transactions on Circuits and Systems* 22.6 (1975):
  504-509.
- Nagel, Laurence W. *SPICE2: A Computer Program to Simulate Semiconductor Circuits*.
  University of California, Berkeley, 1975.
- Gear, C. William. *Numerical Initial Value Problems in Ordinary Differential
  Equations*. Prentice-Hall, 1971.
- Cortes Garcia, Isabel, Herbert De Gersem, and Sebastian Schoeps. "A Structural
  Analysis of Field/Circuit Coupled Problems Based on a Generalised Circuit Element."
  *Numerical Algorithms* 79 (2018): 373-394.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench mna
```
