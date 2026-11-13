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
bytes remain outside total-work accounting. These are field-image operations;
validated evolution and general continuity/existence/uniqueness certificates
remain open.

Build on Rust 1.95 and run the time-dependent oscillator example:

```sh
cargo test -p hyperdynamics --all-features
cargo run -p hyperdynamics --features field --example expression_field
```

The default feature set is empty. `dispatch-trace` enables underlying exact
arithmetic instrumentation when `field` is enabled.
