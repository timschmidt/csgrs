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
RK proposals, wrapping control, broader reachability and whole-resource bounds
remain open.

Build on Rust 1.95 and run the time-dependent oscillator example:

```sh
cargo test -p hyperdynamics --all-features
cargo run -p hyperdynamics --features field --example expression_field
```

The default feature set is empty. `dispatch-trace` enables underlying exact
arithmetic instrumentation when `field` is enabled.
