# Reference and performance audit

This audit maps every source in the README reference section to the implemented
HyperCircuit boundary. Performance changes were retained only when the release
benchmark improved and the exact behavior remained covered by tests.

## Results

Measurements use `cargo bench --bench mna --offline` on the same machine and
release profile. The original one-node benchmark measured 1.038 microseconds
per stamp-and-replay iteration and 426 nanoseconds per circuit-carrier lowering.
After the retained assembly changes, three runs measured 858--881 nanoseconds
and 292--338 nanoseconds, with medians of 858 and 299 nanoseconds respectively.
That is approximately a 17% stamp-and-replay improvement and a 30% carrier
lowering improvement. A new 16-node, 30-stamp assembly fixture measures roughly
10.9--11.3 microseconds and protects the larger-system path from regressions.

The retained implementation:

- reserves the complete MNA unknown vector and moves caller-owned net IDs into
  it instead of cloning them;
- indexes net-voltage and branch-current identities separately with borrowed
  keys, avoiding a cloned `MnaUnknown` and cloned string for every stamp lookup;
- resolves each companion stamp's endpoints once for both its conductance and
  history-current contributions; and
- rejects duplicate net-voltage or branch-current unknowns before assembling a
  malformed singular system.

The generated KCL reference test exercises all implemented linear stamp kinds
together over 256 parameter combinations. The opt-in `dispatch-trace` test also
shows that accepted exact MNA replay performs sign/zero certification without
requesting approximation or producing an unknown fact.

## Source-by-source audit

### Ho, Ruehli, and Brennan, *The Modified Nodal Approach to Network Analysis*

The paper's central construction is already the crate's linear core: non-ground
node voltages are the primary unknowns, while voltage sources introduce branch
currents and corresponding constitutive rows. Conductance, current-source,
voltage-source, and controlled-source stamping preserve that block structure.
Its emphasis on setup cost, matrix dimension, nonzeros, and fill motivated the
identity-resolution optimization and the larger assembly fixture. Duplicate
unknown rejection now protects the square-system structure assumed by MNA.

The paper's sparse pivoting and fill analysis are not reproduced in this dense,
small-system carrier. Solver ordering and sparse factorization remain owned by
`hypersolve` or a future sparse adapter rather than being partially duplicated.

### Nagel, *SPICE2: A Computer Program to Simulate Semiconductor Circuits*

SPICE2 motivates explicit separation of device topology, linearized stamps,
transient companion models, nonlinear iteration, and numerical solution. The
crate already exposes companion conductance/history-current stamps, nonlinear
device and event reports, and distinct transient policies. The retained
companion endpoint reuse removes repeated topology lookup within one stamp.

Reusable symbolic sparse topology, Newton iteration, timestep control, and full
device laws would require a materially larger simulator API. They remain named
adapter boundaries instead of being represented by an incomplete dense-path
optimization.

### Yap, *Towards Exact Geometric Computation*

Yap's construction-versus-decision separation applies beyond geometry here:
numeric engines may construct candidate circuit states, but exact residuals
decide acceptance. `LinearMnaSystem::replay_candidate` delegates that decision
to `hypersolve` and HyperReal certification. The feature-gated dispatch trace
provides executable evidence that the exact rational fixture does not silently
fall back to approximate arithmetic.

### Cortes Garcia, De Gersem, and Schoeps, *A Structural Analysis of
Field/Circuit Coupled Problems Based on a Generalised Circuit Element*

The paper treats a field/circuit coupling as a DAE obtained by joining MNA with
discretized field equations. HyperCircuit therefore keeps electrical, thermal,
and electromechanical ports explicit and lowers coupled residual blocks into
`hypersolve` without taking ownership of field state. The electrothermal RC
fixture records both its physical handles and exact constitutive evidence.

Differential-index analysis and generalized field elements cannot be inferred
from the current scalar fixture, so no unsupported index classification or
field solve was added.

### LLNL SUNDIALS IDA

IDA solves residual systems of the form `F(t, y, y') = 0` using variable-order
BDF methods and approximate nonlinear/linear solves. This supports the existing
`IdaDaeAdapter` policy and the rule that adapter results remain proposals until
their circuit-owned residuals replay. HyperCircuit does not claim to implement
IDA's history, consistent-initial-condition calculation, Newton iteration,
linear solvers, or error control; those remain external adapter responsibilities.

## Rejected trials

- Replacing the ordered borrowed-key maps with hash maps regressed the small
  stamp-and-replay fixture to 951--955 nanoseconds and carrier lowering to
  383--387 nanoseconds, while leaving the larger fixture effectively unchanged.
  The ordered maps were restored.
- Rewriting the electrothermal equations solely to use borrowed arithmetic was
  neutral to slightly slower (1.184--1.187 microseconds versus 1.176--1.183
  microseconds immediately before the trial). The original expressions were
  restored.
- A reusable symbolic-topology object could avoid more work across transient
  timesteps, but the present stamps own both topology and values. Adding such an
  object without a parameter-update API would duplicate state and invite stale
  stamps, so it was not retained as a speculative optimization.

## Validation

The audit is validated with default and all-feature tests, generated property
tests, the dispatch trace, formatting, Clippy with warnings denied, rustdoc with
warnings denied, and the release benchmark. The lockfile was refreshed offline
to match the current local HyperReal dependency.
