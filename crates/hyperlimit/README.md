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

## Hyperreal Integration

The `StructuralScalar` and `PredicateScalar` traits model the facts a
Hyperreal-aware backend could expose:

- known sign
- exact zero / provably nonzero
- exact vs approximate state
- rational-only state
- magnitude bounds
- refinement until sign is decided

The initial `backend::hyperreal` module is a compile-time sketch. It names the
adapter shape without forcing this crate to depend on a particular `hyperreal`
API before that API is stable.

## Status

This is a starter architecture. The first concrete backend supports `f64` and
uses conservative filters plus explicit `Unknown` results. Future work should
wire in `robust`, interval arithmetic, and Hyperreal-native refinement.
