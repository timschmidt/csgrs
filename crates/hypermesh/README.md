<h1>
  hypermesh
  <img src="./doc/hypermesh.png" alt="hypermesh logo" width="144" align="right">
</h1>

`hypermesh` is the experimental exact 3D mesh-topology crate for the Hyper
stack. Its primary type is `ExactMesh`: exact vertices, triangle topology,
retained validation facts, exact bounds, construction provenance, and cached
predicate evidence.

`hyperreal` is the canonical geometry scalar. Primitive floats are accepted only
at explicit import boundaries where approximation policy and provenance are
recorded.

## Scope

Mesh booleans combine geometry, topology, and cleanup in one fragile pipeline.
Nearly coplanar triangles, vertices on edges, duplicate faces, slivers,
non-manifold input, and tolerance-based repair can all change output topology.
`hypermesh` keeps those decisions exact by retaining the source facts used by
validation, broad phase, face-pair classification, coplanar arrangements, split
planning, winding, and mesh assembly.

Application adapters and operation routing belong above this crate. `hypermesh`
provides mesh-kernel storage, replayable acceleration facts, low-level exact
algorithms, typed blockers, and `ExactMesh` convenience methods required by
downstream CSG layers.

## Public Surface

`ExactMesh` is the entry point: it owns exact vertices, triangle topology,
validation facts, bounds, and construction provenance. It also carries the
convenience methods downstream CSG layers need: `union`, `intersection`,
`difference`, `xor`, `transform`, `inverse`, and `with_arrangement_view`.

Supporting root exports are deliberately small. `ExactMeshError` and
`ExactMeshBlocker` report invalid input, unsupported exact topology, stale
replay, and construction blockers with provenance where available.
Boundary-allowed input uses named `ExactMesh` constructors instead of a public
policy object.

Borrowed queries start from `ExactMesh::view()`. Mesh, triangle, face, and edge
views avoid cloning mesh storage; prepared views reuse replay-validated
broad-phase facts and stream candidate face pairs with fallible early-stop
support. `ExactMesh::with_arrangement_view` exposes borrowed arrangement queries
for algorithms that need retained topology without cloning arrangement storage
or naming an owned arrangement type.

Retained graph, arrangement, cell-complex, winding, and shortcut evidence remain
kernel internals unless a borrowed view is needed for exact query reuse.

## Precision Model

Geometry is stored as `hyperreal::Real`. Finite `f64` coordinates enter through
`ExactMesh::from_lossy_f64_triangles`, which imports by dyadic lifting with
lossy provenance recorded explicitly; integer-grid input is lifted directly into
exact `Real` values. Retained face planes keep unnormalized determinant
coefficients instead of unit normals.

Exact predicates and replayable facts are the source of topology decisions.
Unresolved coplanar, boundary, winding, or construction state is returned as a
typed blocker rather than patched with a tolerance.

## Performance Model

The performance direction is broad-phase pruning plus exact local decisions.
Retained bounds, prepared views, streamed face-pair classification, split plans,
support intervals, coplanar arrangements, and borrowed views narrow work before
expensive predicates or topology rebuilds.

One-shot booleans should be driven by measured kernel stages: broad phase,
narrow classification, split planning, local arrangements, winding/ownership,
triangulation, and materialization. The generic fallback remains exact
arrangement/cell-complex construction with winding evidence and CDT remeshing
for difficult inputs.

## Status

The default crate root centers on `ExactMesh`, typed kernel errors, and borrowed
mesh/triangle/face/arrangement views. Unsupported boolean, intersection, or
simplification topology is reported as a blocker instead of falling back to
tolerance-based geometry.

## Installation

```toml
[dependencies]
hypermesh = "0.3.0"
```

## Usage

The exact-facing path is the preferred boundary for new code:

```rust,ignore
use hypermesh::ExactMesh;

let mesh = ExactMesh::from_i64_triangles(
    &[
        0, 0, 0,
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
    ],
    &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
)?;

let view = mesh.view();
assert_eq!(view.face_count(), 4);
assert!(view.is_closed_manifold());
mesh.validate_retained_state()?;
```

Named booleans are mesh methods:

```rust,ignore
let union = left.union(&right)?;
let intersection = left.intersection(&right)?;
let difference = left.difference(&right)?;
let xor = left.xor(&right)?;
let inverse = left.inverse()?;

union.validate_retained_state()?;
```

## References

- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*
  7.1-2 (1997): 3-23.
- Shewchuk, Jonathan Richard. "Adaptive Precision Floating-Point Arithmetic and
  Fast Robust Geometric Predicates." *Discrete & Computational Geometry* 18.3
  (1997): 305-363.
- Guigue, Philippe, and Olivier Devillers. "Fast and Robust Triangle-Triangle
  Overlap Test Using Orientation Predicates." *Journal of Graphics Tools* 8.1
  (2003): 25-42.
- Boissonnat, Jean-Daniel, Olivier Devillers, Sylvain Pion, Monique Teillaud,
  and Mariette Yvinec. "Triangulations in CGAL." *Computational Geometry*
  22.1-3 (2002): 5-19.
- Requicha, Aristides A. G. "Representations for Rigid Solids: Theory, Methods,
  and Systems." *ACM Computing Surveys* 12.4 (1980): 437-464.

## Development

Useful local checks:

```sh
cargo check --all-targets
cargo test --test kernel_exact_mesh
cargo test bounds::tests
cargo check --manifest-path fuzz/Cargo.toml
cargo fuzz run exact_mesh_input
cargo fuzz run exact_integer_mesh_input
```
