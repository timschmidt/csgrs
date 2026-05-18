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
- [hypersolve](https://github.com/timschmidt/hypersolve): future nonlinear residual,
  DAE, and candidate-certification surface.
- [hyperparts](https://github.com/timschmidt/hyperparts): part, terminal, package, and
  source-attributed electrical metadata.
- [hyperpath](https://github.com/timschmidt/hyperpath) and
  [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB trace/routing and release
  package evidence used by coupled fixtures.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): materials, thermal
  carriers, and physical coupling reports.

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

## Precision Model

Circuit truth is represented with `hyperreal::Real`. Primitive floats should appear only
inside named numeric adapters, imported data, diagnostics, or external solver bridges.
The current linear path constructs exact dense MNA rows and replays exact residuals for
candidate vectors. For nonlinear and coupled systems, the crate stores exact parameters,
domains, piecewise-linear segments, slope facts, and event policy even when the eventual
solver is approximate.

Unknown replay is a valid result. It is preferable to accepting a candidate whose
precision boundary was not certified.

## Performance Model

The present implementation favors clear carriers and exact replay over high-performance
sparse solving. Performance work is expected to come from:

- keeping IDs stable so sparse adapters can map rows and unknowns without rebuilding
  semantic context;
- separating stamp construction from residual replay;
- carrying model and event facts so future solvers can avoid unnecessary nonlinear work;
- using dense exact systems only for small fixtures and certification paths;
- making large sparse, transient, DAE, and field solvers explicit adapters with reports.

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
    AdapterKind::ExactDenseMna,
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
let replay = system.replay_residuals(&[Real::zero()])?;
assert!(replay.accepted);
```

Other major surfaces follow the same pattern: `NonlinearDeviceReport` records device
law and event policy before Newton-style proposal engines exist, while
`ElectrothermalRcReport` and `CircuitAdapterReport` keep coupled physics and numeric
adapter status separate from circuit truth.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench mna
```
