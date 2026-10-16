# Affected consumers identified before implementation

`ScalarMesh<A>` owns the conversion of native Boolean errors for all four
adapters (`RawReal`, `F32`, `F64`, `I128`). `AdapterError` already has a
`Construction(ValidationError)` variant, unchanged display forwarding and an
`Error::source` implementation. The eight mesh Boolean entry points currently
erase that structure with `Validation(String)`.

The prepared Rust public probe exercises every Boolean and all four families.
Invalid-index geometry is paired with both canonical empty and a valid cube,
in both operand orders, through strict convenience and both context variants.
A cube translated by an unresolved trigonometric zero exercises strict
undecided errors. Native category and display text are independently checked
before requiring the adapter's matching typed cause. This probe has not yet
been executed; it is not evidence of a reproduced failure or a passing fix.

`crates/csgrs-ffi/src/lib.rs` uses these adapters. Its existing
`status_for_error` already maps `Construction`'s native categories to
`PredicateUndecided`, `Unbounded`, `ResourceExhausted`, `Unsupported`, or the
existing invalid-input status. No new C enum value is needed. Restoring the
typed adapter cause intentionally changes an undecided Boolean's C status from
generic `ValidationFailed` to the existing `PredicateUndecided` (5).

The C++ wrapper preserves the status in `csgrs::error::status()` and the Python
ctypes wrapper in `CsgrsError.status`. These are affected public consumers and
must be exercised against the actual newly built library, including exception
messages, ownership, empty/invalid distinction and successful recovery. Existing
`tests/ffi/empty_composition.*` fixtures supply malformed open-mesh/empty cases.
Before using a rotated-cube Boolean as a new undecided binding fixture,
reproduce its current native and binding outcomes; an assumption is not a test.

The generated WASM `MeshJs` implementation calls native `SolidExt` Booleans
directly rather than `ScalarMesh`. Its certainty consumers belong to the E12
qualification; they do not demonstrate the scalar-adapter diagnostic repair.
No adapter production code or binding code has changed in this preparatory
scope record. Existing scalar proof qualification must finish before executing
the new serial diagnostic/build phases.
