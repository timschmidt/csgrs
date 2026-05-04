# predicated

`predicated` is a geometry-oriented predicate layer for robust 2D/3D
classification. It is designed to sit between semantics-rich scalar crates such
as `hyperreal`, vector/matrix crates such as `realistic_blas`, and application
geometry kernels such as `csgrs`.

The core idea is simple: robust geometry should not treat every scalar as an
opaque number. If a value can expose structural facts such as known sign, exact
zero, rational-only status, exactness, magnitude bounds, or refinement state,
predicate code should exploit those facts before paying for expensive fallback
paths.

## Architecture

Predicates are evaluated as an explicit escalation pipeline:

1. Structural checks on intermediate values and inputs.
2. Cheap floating or bound-based filters.
3. Exact-ish backend paths when the scalar can provide them.
4. Robust or adaptive fallback implementations.
5. Targeted refinement only when the sign or classification is still unknown.

For many geometry predicates, the important question is not "what is the full
determinant?" but "can we prove its sign or prove that it is zero?" The public
API reflects that distinction.

## Current Crate Layout

- `scalar`: scalar capability traits and structural facts.
- `predicate`: predicate outcomes, signs, escalation metadata, and policies.
- `filter`: cheap filtering helpers.
- `orient`: `orient2d`, `orient3d`, and determinant-sign helpers.
- `plane`: plane-side and point/plane classification helpers.
- `classify`: geometry classification enums.
- `backend`: optional backend adapters and integration sketches.
- `error`: shared error/result types.

## Why Not Just Use `robust`?

Pure robust predicate crates are excellent fallback engines, but they generally
start from opaque primitive coordinates. `predicated` owns the geometry-specific
policy around when to ask for fallback and when to avoid it because the scalar
layer already knows enough.

For example, a `hyperreal` value may know that a determinant term is exactly
zero, rational-only, or provably nonzero without expanding the entire expression.
That can avoid unnecessary adaptive expansion work.

## Why Not Put This in `realistic_blas`?

`realistic_blas` should own vector and matrix operations. This crate owns
geometry robustness policy: sidedness, orientation, coplanarity, splitting, BSP
classification, and intersection decisions. Keeping this layer separate avoids
turning the linear algebra layer into an application-specific geometry kernel.

## Hyperreal and realistic_blas Integration

The `StructuralScalar` and `PredicateScalar` traits consume facts exposed by
`hyperreal` and forwarded through `realistic_blas`:

- known sign
- exact zero / provably nonzero
- exact rational state
- magnitude bounds
- refinement until sign is decided

Enable the `hyperreal` feature to implement the predicate scalar traits for
`hyperreal::Real`. Enable the `realistic-blas` feature to implement them for
`realistic_blas::Scalar<B>`.

## Status

This is a starter architecture. The first concrete backend supports primitive
floating point and uses conservative filters plus explicit `Unknown` results.
Enabling the `robust` feature wires in adaptive orientation fallback for finite
`f64`-convertible coordinates, plus incircle/insphere fallback. Enabling the
`geogram` feature uses the `dev-rust-port` branch of `geogram_predicates` for
orientation fallback and is preferred for orientation when both fallback
features are enabled. Enabling `hyperreal` or `realistic-blas` uses structural
sign, zero, magnitude, and bounded refinement facts before fallback. Future work
should add interval arithmetic.
