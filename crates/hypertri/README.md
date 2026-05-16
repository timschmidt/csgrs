# hypertri

`hypertri` is a triangulation crate for the hyperreal geometry stack. It owns
earcut-style polygon triangulation and incremental Delaunay / constrained
Delaunay topology, with irreversible topology decisions routed through exact
predicate helpers over `hyperreal::Real`.

The crate treats `f64` as an interop boundary. Finite `f64` coordinates are
validated and lifted into exact hyperreal-backed coordinates before topology is
decided. The default feature set exposes the exact hyperreal APIs; enable
`f64-interop` only at IO, rendering, or compatibility boundaries.

## WASM Demo

The deployed WASM app is available at
<https://timschmidt.github.io/hypertri/>.

## Current Status

Implemented and tested:

- Exact point and polygon input types backed by `hyperreal::Real`.
- Ring normalization from flat vertices plus `hole_indices`.
- Polygon structural facts for vertex/ring counts, known degenerate and
  axis-aligned edges, exact-rational coordinate summaries, symbolic dependency
  summaries, signed ring area, and local turn consistency.
- Earcut-style triangulation for simple and holed polygons.
- Local-intersection curing and split fallback for difficult earcut inputs.
- Exact incremental Delaunay triangulation for point sets.
- Closed-ring constrained triangulation for one exterior ring plus holes.
- Constraint recovery by exact edge flips.
- Exact vertex insertion at proper constraint intersections.
- Constraint splitting at inserted intersection vertices and existing collinear
  vertices.
- Re-legalization of unconstrained edges with exact in-circle predicates.
- Runtime polygon algorithm selection when `runtime-select` is enabled.
- Optional `f64` entry points that reject non-finite input and exact-lift finite
  coordinates before topology is decided.
- WASM UI example built with Trunk and deployed through GitHub Pages.

For constrained output, `constraints()` reports the caller's original
constraints and `constraint_edges()` reports the planarized protected
subsegments that are actually present as triangulation edges. Exact results
provide `validate()` for topology checks and
`validate_unconstrained_edges_are_delaunay()` for the local constrained
Delaunay legality check on unprotected interior edges.

Still incomplete:

- Full DCEL cavity deletion/remeshing is not yet ported.
- The constrained triangulation path is intentionally conservative around cases
  that need more complete cavity rebuilding or richer exact object facts.
- Common-scale / homogeneous rational polygon facts are planned but not yet
  first-class inputs.
- `serde` is reserved but not yet a public topology serialization surface.

## API Shape

The exact API is the default:

```rust
use hypertri::{Point2, Real};

fn main() -> hypertri::Result<()> {
    let points = vec![
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(1), Real::from(0)),
        Point2::new(Real::from(0), Real::from(1)),
    ];

    let triangles = hypertri::earcut(&points, &[])?;
    assert_eq!(triangles.len(), 3);
    Ok(())
}
```

The optional `f64` module is for IO, rendering, tests, and compatibility:

```rust
fn main() -> hypertri::Result<()> {
    let triangles = hypertri::f64::earcut(
        &[[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
        &[],
    )?;

    assert_eq!(triangles.len(), 3);
    Ok(())
}
```

Those APIs lift finite primitive floats into exact `Real` values before
topology branches run.

## Feature Flags

- `earcut`: polygon triangulation by ear clipping.
- `cdt`: Delaunay and constrained Delaunay APIs.
- `runtime-select`: runtime polygon algorithm selection when multiple
  algorithms are compiled.
- `f64-interop`: opt-in plain `f64` entry points that exact-lift finite input.
- `serde`: reserved for public topology serialization.

`spade` and `earcutr` are source references for the port. They are not
production dependencies of `hypertri`.

## Semantic Boundary

`hypertri` owns triangulation topology: polygon normalization, ring and hole
inputs, linked earcut nodes, CDT records, constraint graphs, protected edge
metadata, runtime algorithm selection, output validation, and triangulation
result records. Exact orientation, ring area, local turn, and in-circle signs
are delegated to `hyperlimit`, while polygon object facts consume scalar
summaries from `hyperreal::Real`.

Structural metadata should be retained when it is cheap to discover: source
vertex ids, duplicate classes, collinear chains, exact ring area signs, local
turn consistency, integer-grid, dyadic-scale, shared-denominator, symbolic
dependency facts, bounding boxes, convex/reflex bits, constraint-subsegment
provenance, protected-edge flags, and cavity boundary facts. These facts select
faster exact algorithms and reduce candidate sets; they do not permit lossy
topology decisions.

## Testing

The test suite combines fixed adversarial cases with `proptest` fuzz-style
generators over exact integer and rational inputs. The fuzz properties check
topology invariants such as valid triangle indices, non-degenerate triangle
index triples, constrained edges preserved by the accepted CDT subset, and
exact local Delaunay legality on unconstrained interior edges.

Useful local checks:

```text
cargo test
cargo test --features f64-interop
cargo check --manifest-path examples/hypertri_ui/Cargo.toml --target wasm32-unknown-unknown
cargo bench --bench earcut --features earcut,f64-interop
cargo bench --bench delaunay --features cdt,f64-interop
cargo bench --bench exact --features earcut,cdt
```

The GitHub Pages workflow checks out `hypertri`, `hyperreal`, and `hyperlimit`
as sibling repositories because the manifests intentionally use local path
dependencies while the crates are being developed together.

## References

Bareiss, Erwin H. "Sylvester's Identity and Multistep Integer-Preserving
Gaussian Elimination." *Mathematics of Computation*, vol. 22, no. 103, 1968,
pp. 565-578.

Boehm, Hans-J., Robert Cartwright, Mark Riggle, and Michael J. O'Donnell.
"Exact Real Arithmetic: A Case Study in Higher Order Programming." *Proceedings
of the 1986 ACM Conference on LISP and Functional Programming*, 1986, pp.
162-173.

de Berg, Mark, Otfried Cheong, Marc van Kreveld, and Mark Overmars.
*Computational Geometry: Algorithms and Applications*. 3rd ed., Springer, 2008.

Delaunay, Boris. "Sur la sphère vide." *Bulletin de l'Académie des Sciences de
l'URSS. Classe des sciences mathématiques et naturelles*, no. 6, 1934, pp.
793-800.

Lee, Der-Tsai, and Arthur K. Lin. "Generalized Delaunay Triangulation for
Planar Graphs." *Discrete & Computational Geometry*, vol. 1, 1986, pp.
201-217.

Meisters, Gary H. "Polygons Have Ears." *The American Mathematical Monthly*,
vol. 82, no. 6, 1975, pp. 648-651.

Shewchuk, Jonathan Richard. "Adaptive Precision Floating-Point Arithmetic and
Fast Robust Geometric Predicates." *Discrete & Computational Geometry*, vol.
18, no. 3, 1997, pp. 305-363.

Shewchuk, Jonathan Richard, and Brielin C. Brown. "Fast Segment Insertion and
Incremental Construction of Constrained Delaunay Triangulations."
*Computational Geometry*, vol. 48, no. 8, 2015, pp. 554-574,
doi:10.1016/j.comgeo.2015.04.006.

Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*,
vol. 7, nos. 1-2, 1997, pp. 3-23.
