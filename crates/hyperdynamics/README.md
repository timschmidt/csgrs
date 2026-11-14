# Hyperdynamics

Owned vector fields with explicit time/state interpretation and native interval
images. Hyperdynamics owns field semantics; Hypersolve validates expressions and
symbolic derivatives, Hyperlattice carries Cartesian vectors/matrices, and
Hyperinterval encloses scalar arithmetic. There is no geometry-layer dependency.

Enable `field` for the sealed `EnclosedVectorField` interface and
`ExpressionField<N>`. A `FieldDefinition` supplies N output expressions, one time
variable and N distinct state variables. Source bindings may occur in any order.
The immutable identity includes the full source and time/state mapping. Enable
`serde` to serialize that identity; deserialize into `FieldDefinition` and rebuild
through `ExpressionField::from_definition`, which revalidates the source and
regenerates its Jacobian. Executable fields and validity flags are not deserialized.

Images consume `BoxN<DyadicInterval, N>` and return native interval vectors;
state Jacobians return native interval matrices. Original source domain checks
remain active for Jacobians, including zero partials. Empty inputs produce
canonical empty products. Zero-dimensional unit products remain distinct from
empty sets. A valid value image can coexist with an uncertified derivative image.

Pass one `EnclosureBudget`, configured by an explicit `EnclosurePolicy`, through
construction and nested calls to aggregate structural and dyadic arithmetic work.
Precision specifies a rounding grid, not an output-width guarantee. Source
serialization, scalar payload cloning, Hyperreal constant refinement and heap
bytes remain outside total-work accounting. These counters do not yet provide
whole-work or total-heap bounds.

Enable `taylor-model` for `field.taylor_model(request, budget)`. The request
specifies a time/state box and total polynomial degree; each exact midpoint
defines the center. Hypersolve constructs exact coefficients and a certified
finite remainder. The result retains the original field identity, request,
coefficient/remainder evidence and native image. `enclose_on` consumes subdomains
with the retained grid and global remainder, preserving empty/unit products.
Source replay reconstructs the proof. This certifies a field image; flow
existence, model composition and wrapping control require additional work.

```sh
cargo run -p hyperdynamics --features taylor-model --example taylor_model
```

`ExpressionField::certify_step` checks a proposed Picard self-map and contraction,
returning source-bound tube and endpoint evidence or a typed rejection.
`ExpressionField::integrate` searches bounded expansions and step halvings on a
dyadic horizon. It retains every attempt, certified prefix and whole-request
status; only a completed request exposes its final endpoint.

For exact rational physical times, call `field.on_time_span(start, end, budget)`
and integrate the resulting `RationalTimeField`. It retains the original field
and exact affine clock `t = start + (end-start)*tau`, and certifies the transformed
field on `tau` in `[0,1]`. `EvolutionOptions::maximum_step` uses these normalized
tau units: on `[1/3,2/3]`, a cap of `1/4` means at most `1/12` physical time.
Physical segment endpoints stay rational; state enclosures remain outward dyadic
bounds. Final physical-time materialization must succeed before completion.
The endpoint enclosure can seed another horizon. A zero horizon is an identity
with no ODE existence claim; empty and stopped outcomes remain distinct.

With `serde`, serialize a `RationalTimeField` and deserialize into
`RationalTimeDefinition`, then rebuild with `RationalTimeField::from_definition`.
Replay revalidates the original source and regenerates the clock, transformation
and derivatives. Generated proof flags are never trusted as certificates.

Enable `rk-proposal` for `field.rk4_proposer(budget)`. Its reusable primitive view
retains the exact source. `propose(request, options, budget)` records four RK4
stages and a padded candidate box; padding is a search heuristic without an error
bound. Consume the candidate with `certify(budget)` and inspect `proof().status()`:
only the existing exact Picard verifier can certify a tube or endpoint. Failed
proposals retain the original request and partial stage trace. Rebuild from the
field definition, request and options for replay; numerical traces are not proof.
The native horizon integrator remains available independently. Automatic RK step
selection, wrapping control, broader reachability and whole-resource bounds remain
open.

Build on Rust 1.95 and run the time-dependent oscillator example:

```sh
cargo test -p hyperdynamics --all-features
cargo run -p hyperdynamics --features field --example expression_field
cargo run -p hyperdynamics --features rk-proposal --example rk_proposal
```

The default feature set is empty. `dispatch-trace` enables underlying exact
arithmetic instrumentation when `field` is enabled.

Enable `reachability` for `field.reach(request, policy, budget)`. The request gives
an exact dyadic horizon, an initial box and nonnegative singleton `epsilon`.
Only `end_cover()` on a completed report exposes a finite endpoint union `C`:
`R ⊆ C ⊆ R + [-epsilon, epsilon]^N`, where `R` is the exact reachable set.
This is an infinity-norm bound in the supplied state-coordinate units, not an
exact reachable-set or per-trajectory pairing claim. Identity and empty requests
retain their exact set meanings. Zero epsilon need not complete.

The search partitions the widest initial coordinate exactly, choosing the first
axis on ties and visiting the lower child first. Small cells may instead halve
the time cap. Every cell, failed attempt and exact source identity is retained.
Native certified prefixes are consumed explicitly: their local one-segment-limit
status is preserved. Each continuation box is rounded outward on the enclosure
policy's grid, with checked inclusion. The final transfer and endpoint diameter
must pass before whole-horizon success. The original `integrate` strategy remains
available; reachability selects displacement-based untrusted candidate padding
and uses the same Picard verifier.

`ReachabilityPolicy` bounds cells, depth, whole-cell evolution attempts and time
refinements. Each attempt performs at most `max(1, evolution.maximum_steps)`
native prefix searches. All calls share the supplied budget. These bounds retain
the existing exclusions for scalar refinement, total CPU work and heap bytes.
A rounding floor, a mathematical/domain failure or an exhausted bound yields a
typed stopped report with no whole-request cover. Rebuild the field and rerun
source/request/policies for replay; reports are not deserialized as proof.

```sh
cargo run -p hyperdynamics --features reachability --example reachability
```

Run `cargo run -p hyperdynamics --features taylor-affine --example taylor_affine`
for source-bound linear observables of a nonlinear field. The example retains
shared Taylor coefficients through an exact output map, then explicitly projects
the result for a further field operation. It does not transform an ODE proof or
replace the authoritative model with its Cartesian image.

### Fixed state coordinates

With `coordinates`, `ExpressionField::in_coordinates` certifies `x = c + A*y`
and constructs `y' = A^-1*f(t,c+A*y)` using Hypersolve. Time and the ordered state
interpretation are retained. Its transformed field supports existing image,
Jacobian, Taylor (when enabled), Picard and evolution routines.
`CoordinateField::certify_step` additionally retains the chart and transports a
completed native Picard tube and endpoint to original-coordinate enclosures.
Original initial states are the exact correlated set `c + A*Y`; its Cartesian
envelope is only an explicit outer projection. Final transport exhaustion retains
the native report but cannot publish whole-step success. Replay validates source,
inverse and derivatives again. Fixed charts do not imply adaptive QR selection,
a whole-resource bound or automatic wrapping reduction.

Run `cargo run -p hyperdynamics --features coordinates --example coordinates`
for an exact scaling of `x1' = 128*x2, x2' = 0` that admits a half-unit step after
the original-coordinate contraction check rejects it.

The opt-in `taylor-polynomial` feature forwards Hypersolve polynomial arithmetic
on existing native Taylor models. The original field identity and time/state
binding remain with the field model; output-image arithmetic is not an ODE flow
or coordinate-change certificate.

Enable `taylor-flow` for `field.flow_taylor(order, budget)`. The prepared program
constructs source-bound normalized Lie coefficients through the next order. Its
`certify_step(proposal, budget)` first proves Picard existence/uniqueness, then
requires the original Taylor layer's analytic-domain proof over the complete
time/tube product. Initial coefficients and the whole next-order remainder give
certified trajectory and endpoint enclosures. Every contribution and both source
proofs remain inspectable; generated endpoints can seed later steps. Replay the
field definition, order and proposal. This is a forward-step verifier, not an
automatic order/step controller or adaptive wrapping algorithm.

With `coordinate-transition`, `CoordinateField::transition_to` prepares direct endpoint transport between two validated charts of the same field. `enclose` retains the exact relation and input beside its outer box, which can enter the destination chart’s next step. Frame selection and QR wrapping control remain separate work.

With `qr-coordinates`, `ExpressionField::in_qr_coordinates` constructs an exact orthonormal state chart from a basis and origin without a supplied inverse. The retained factorization and chart support existing Picard, Taylor, finite-horizon and direct endpoint-transport APIs. Frame selection remains caller-controlled.

The optional `taylor-evolution` feature composes validated trajectory Taylor endpoints over a finite dyadic horizon. It retains each bounded Picard search and the full Taylor remainder, feeds refined endpoints into subsequent steps, and distinguishes a certified prefix from whole-horizon completion.

The optional `adaptive-taylor` feature adds
`field.adaptive_taylor(AdaptiveTaylorOptions::default()).integrate(request, budget)`.
It tries a finite ascending range of Taylor orders on each original certified Picard
proposal, checking every coordinate's **absolute endpoint remainder contribution**
against its exact nonnegative singleton limit. The first satisfactory order advances
the state. If all orders fail quality, bounded exact halving retries; accepted durations
may double for the next proposal, capped by the configured maximum and exact horizon.
Native existence-search and quality-search halving limits remain separate.

The report retains all prepared programs, native searches, Taylor evidence, failed
quality comparisons and accepted choices. Only a completed run exposes the whole-horizon
endpoint; a stopped prefix remains separately queryable. Native/Taylor domain or resource
failures stop with their original cause. Empty and identity cases validate configuration
and time first. Limits concern local truncation contributions, not global solution error,
initial-state uncertainty or equality with the exact reachable set. Selection does not
claim optimal order, minimum runtime or adaptive frame/QR control.

Pass one budget through construction and integration. Program preparation is lazy and
cached once per distinct order within a run; preparing different orders still uses the
existing coefficient generator independently. Structural record checks and original
graph/native counters are shared; exact scalar payload/refinement, nested retained graph
size, vector capacity, whole heap and within-scalar cancellation remain outside a total
resource claim. Replay the field definition, request and all selection options to rebuild
proofs; serialized statuses and cached enclosures are not certificates.
