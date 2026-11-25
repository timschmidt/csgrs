# Hyperdynamics

Owned vector fields with explicit time/state interpretation and native interval
images. Hyperdynamics owns field semantics; Hypersolve validates expressions and
symbolic derivatives, Hyperlattice carries Cartesian vectors/matrices, and
Hyperinterval encloses scalar arithmetic. There is no geometry-layer dependency.

With `field`, this example certifies `x'=2`, then consumes its endpoint in
another horizon. The example also uses Hyperinterval with `dyadic`, Hyperlattice
Interval with `dyadic`, and Hypersolve with `interval-program`.

<!-- quickstart:start -->
```rust
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionOptions, EvolutionRequest, EvolutionStatus},
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let time = SymbolId(0);
    let state = SymbolId(1);
    let source = ExpressionProgram::compile(
        &[Expr::int(2)],
        &[time, state],
        &mut ExpressionBudget::new(1000, 10000),
    )?;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let field = ExpressionField::<1>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![state],
        },
        &mut budget,
    )?;
    let point = |n: i64| DyadicInterval::point(n.into(), 0);
    let mut initial = BoxN::new([point(0)?])?;
    for start in 0..2 {
        let report = field.integrate(
            EvolutionRequest {
                start: point(start)?,
                end: point(start + 1)?,
                initial,
            },
            EvolutionOptions::default(),
            &mut budget,
        );
        assert!(matches!(report.status(), EvolutionStatus::Completed));
        assert_eq!(report.identity(), &field.identity());
        initial = report.end_enclosure().expect("completed horizon").clone();
        assert_eq!(initial.lanes(), Some(&[point(2 * (start + 1))?]));
    }
    Ok(())
}
```
<!-- quickstart:end -->

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

From the workspace root, run `python3 scripts/interval_coverage.py --package hyperdynamics`
with the active Rust toolchain's `llvm-tools-preview` component installed.
The command runs the all-feature native library and integration tests, checks
the source inventory, and writes a per-file report under the Cargo target
directory. It exits unsuccessfully if an instrumented production metric is
below 100%. For branch counts, use nightly Rust with
`RUSTFLAGS=-Zcoverage-options=branch`.

To run the numerical and replay suites on WASI Preview 1, use Node.js
24.22.3 and Python 3 from the workspace root. Choose a new output directory
for each run.

```sh
rustup target add --toolchain stable wasm32-wasip1
python3 crates/hyperinterval/conformance/wasi-dynamics.py --toolchain stable --output target/dynamics-wasi
```

The runner matches the native and guest test inventories. Each fresh-process
replay uses the original native parent to check evidence computed from its
inputs in a new WASI instance. The small-stack destruction test runs on a
native thread, and its guest run checks the platform's thread-creation limit.
The output directory contains the test logs, child execution counts and suite
inventory.

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

Enable `flow-sensitivity` for certified initial-state derivatives of a native
evolution. Wrap the report in an `Arc` and call
`report.sensitivity(options, budget)`. The result encloses `D_x0 φ(t, t0, x0)` for
every initial state in the request's box. `end_matrix()` gives it at the
requested horizon, and each step gives it at its segment end and across the
whole segment. The derivation reuses the source's certified Picard Jacobians and
contraction bounds, without evaluating the field again:

- Each segment sums time-ordered Peano–Baker terms `(h·A)^k/k!` as independent
  interval products, so no commutativity is assumed.
- The omitted terms are bounded by `q^(p+1)/(p+1)!/(1-q)`, where
  `q = h·sup‖A‖∞ < 1` is the segment's certified contraction.
- Local derivatives compose on the left of earlier ones.

The same call works on the other evolutions when their features are enabled:
`FlowTaylorEvolution`, `AdaptiveTaylorEvolution`, the fixed-chart
`CoordinateEvolution`, `QrEvolution` and the affine-noise
`FlowNoiseEvolution`. Each Taylor or noise segment keeps the Picard proof its
refinement consumed, and that proof supplies the Jacobian. A noise segment's
proof starts from the box hull of the correlated state, which is often much
smaller than a wrapped native endpoint. On Van der Pol from a `2^-6` box to
`t = 1/4`, every derivative entry is 12–36% narrower than from the native
evolution. A chart
run integrates `y' = B f(t, c + A y)` with an exact `A` and a proved inverse
`B`, so the source-coordinate derivative is `A_k · P(t) · B_0`. Here `P`
composes the chart-coordinate segment derivatives and, at each change of QR
frame, the exact linear part `B_new A_old` of the certified transition. Global
matrices are always in source coordinates. Local matrices are in the segment's
own coordinates. The result's type names its source,
`FlowSensitivity<N, S>`, and `source()` gives that source's own status and
reached time. A stopped QR run contributes only its completed windows.

A stopped source keeps its status: `prefix_end_matrix()` covers its certified
prefix and `end_matrix()` is `None`. An empty request yields the empty matrix
set and a zero-duration request the identity. A source that certified no
segment is an error carrying the source's own failure, and so is any resource, scalar or cancellation failure;
neither publishes partial output, and the source remains available for a retry
with a larger budget. The matrices are Cartesian outer enclosures, not exact
derivative sets.

The example certifies a pendulum's sensitivity to its release angle and speed,
and checks that the derivative's determinant contains 1, as Liouville's theorem
requires for a Hamiltonian flow:

```sh
cargo run -p hyperdynamics --features flow-sensitivity --example flow_sensitivity
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

With `qr-evolution`, `CoordinateField::integrate_qr` selects and certifies a new
frame proposal for each bounded window. For the incoming map `x=c+A*y`, it proposes
`A*(I+h*M)` from midpoint Jacobian data and recenters at `c+A*midpoint(initial)`.
The midpoint is only a proposal: existing exact QR, direct chart transport and
trajectory verification establish every consumed result. `QrEvolutionOptions`
chooses window/native limits and an optional trajectory Taylor order (default3).
Taylor windows retain the existing analytic-domain and full-remainder contract;
`None` selects Picard endpoints. No failed frame or Taylor proof silently falls
back to a weaker success. Original source, selections, relations, native attempts,
coefficients and projections remain inspectable. Both the proposed and retained charts must complete. An exact comparison of their
source-coordinate maximum diameters selects the smaller result (ties select the
recentered proposal); both proofs and the comparison remain available. This local
criterion is not a whole-horizon optimality claim. Only whole-request success
exposes `source_end_enclosure`; stopped windows and certified prefixes are separate.
Replay the original chart, request, options and policy. Frame changes can increase
cost or enclosure width, and a singular Euler-basis proposal stops explicitly.
Exact scalar payload/refinement, nested proof graphs and total heap remain outside
the shared structural/native-arithmetic resource bounds.


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


Enable `taylor-noise` and run `cargo run -p hyperdynamics --features taylor-noise
--example taylor_noise` for repeated affine observations of a nonlinear field.
Hypersolve retains a bounded set of shared monomials using certified contribution
widths and includes every omitted term in distinct residual bounds. The example
checks an exact cancellation and consumes an explicit Cartesian projection in a
further field-image operation while retaining the original source certificate.
This is field-image wrapping control; noise propagation through ODE trajectories
and sensitivity-adaptive noise creation are not implemented by this feature.

A prepared trajectory program can supply any lower polynomial order with
`FlowTaylorProgram::restrict_order(order, &mut budget)`. The result shares the
exact coefficient graphs and derivative requests, and retains the requested
order's mandatory remainder stage. It is an ordinary program accepted by
`certify_step` and whole-horizon Taylor evolution. This only shares preparation;
trajectory existence, analyticity and the complete remainder are still certified
for each supplied domain. Lower orders copy a bounded array of stage handles;
same-order requests share the whole array. No source term or guard is discarded.
Run `cargo run -p hyperdynamics --features taylor-flow --example taylor_orders`.
The adaptive controller still prepares each distinct order independently; callers
can explicitly reuse a prepared higher order for their own finite portfolios.

With `flow-noise`, `FlowNoiseState` retains an exact affine definition of an outer
endpoint set. Pass it to `FlowTaylorProgram::certify_noise_step` with a proposed
Picard tube, end time and retained-column capacity. A complete result can enter
the next step with its shared uncertainty intact. Hypersolve certifies the finite
endpoint polynomial's parameter Taylor model and selects its retained columns;
discarded contributions and the trajectory remainder remain explicit residuals.
Those residuals become distinct independent inputs on the next step. The state
retains the original box and every field/proof association through shared history.

The affine set encloses the reachable set; it does not assert equality with it.
The next Picard check covers every affine parameter combination, even when a
separate endpoint proof supplies a narrower Cartesian box. Failed or exhausted
attempts expose their partial evidence but no new state. This API uses explicit
proposed steps, not an automatic noise-aware horizon controller. The existing
budget exclusions for scalar refinement, total heap and within-scalar
cancellation still apply.

```sh
cargo run -p hyperdynamics --features flow-noise --example flow_noise
```

Enable `flow-noise-evolution` for
`program.integrate_noise(initial, end, options, budget)`. The prepared trajectory
program now searches bounded native step durations and tube candidates while
carrying the complete affine outer endpoint into each next search. Accepted
Picard proofs are reused once; analytic Taylor remainders, selected uncertainty
columns and all discarded contributions remain authoritative. The final step
is capped at the exact requested time.

`FlowNoiseEvolutionOptions` selects the retained-column capacity and existing
native search/segment limits. The report retains every native attempt and
noise refinement. Only `Completed`, validated `Identity`, or `Empty` exposes
`end_state`; `prefix_end_state` explicitly exposes a certified prefix after a
later failure. Empty results retain the requested time and can enter another
horizon. Invalid times/options are checked before empty or identity shortcuts.

All calls share the supplied enclosure budget. Structural history checks and
existing scalar counters do not bound total heap, all retained proof graphs,
scalar payload/refinement or within-scalar cancellation. This controller uses
a caller-selected Taylor order and retained-column capacity; it does not
claim global error control, adaptive frame selection or optimal noise retention.
Replay the original box, field/order history, horizon, options and policy;
serialized result flags are not certificates.

```sh
cargo run -p hyperdynamics --features flow-noise-evolution --example flow_noise_evolution
```
