# hypertri

`hypertri` is a triangulation crate for the hyperreal geometry stack. It ports
earcut-style polygon triangulation and spade-style constrained Delaunay
triangulation into code owned by this crate, with topology decisions routed
through crate-local exact predicates over `hyperreal::Real`.

The crate treats `f64` as an interop boundary. Finite `f64` coordinates are
validated and lifted into exact hyperreal-backed coordinates before topology is
decided. The default feature set exposes the exact hyperreal APIs; enable
`f64-interop` only at IO, rendering, or compatibility boundaries.

The current owned implementation includes simple and holed earcut-style polygon
triangulation with exact local-intersection curing and split fallback, exact
incremental Delaunay triangulation for point sets, and closed-ring constrained
triangulation for one exterior ring plus holes. The CDT path also recovers
constraint segments by exact edge flips, inserts exact vertices at proper
constraint intersections, splits constraints at those vertices and at existing
collinear vertices, and re-legalizes unconstrained edges with exact in-circle
predicates. Full DCEL cavity deletion/remeshing remains the next CDT porting
surface.

For constrained output, `constraints()` reports the caller's original
constraints and `constraint_edges()` reports the planarized protected
subsegments that are actually present as triangulation edges. Exact results
provide `validate()` for topology checks and
`validate_unconstrained_edges_are_delaunay()` for the local constrained
Delaunay legality check on unprotected interior edges.

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
inputs, linked earcut nodes, CDT/DCEL records, constraint graphs, protected edge
metadata, runtime algorithm selection, output validation, and triangulation
result records. Exact orientation and in-circle signs are delegated to
`hyperlimit`, while polygon object facts consume scalar summaries from
`hyperreal::Real`.

Structural metadata should be retained when it is cheap to discover: source
vertex ids, duplicate classes, collinear chains, exact ring area signs,
integer-grid, dyadic-scale, shared-denominator, symbolic dependency facts,
bounding boxes, convex/reflex bits, constraint-subsegment provenance,
protected-edge flags, and cavity boundary facts. These facts select faster
exact algorithms and reduce candidate sets; they do not permit lossy topology
decisions.

## Testing

The test suite combines fixed adversarial cases with `proptest` fuzz-style
generators over exact integer and rational inputs. The fuzz properties check
topology invariants such as valid triangle indices, non-degenerate triangle
index triples, constrained edges preserved by the accepted CDT subset, and
exact local Delaunay legality on unconstrained interior edges.

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
