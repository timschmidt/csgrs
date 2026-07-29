# Hypermesh fuzzing

The targets construct bounded exact-real geometry across rational and symbolic
Hyperreal representations. Primitive floats are not used to choose topology.

The Boolean suite is split by purpose:

- `boolean_pipeline` requires supported boxes, tetrahedra, octahedra,
  subdivided surfaces, disconnected components, and variadic inputs to
  succeed. It checks every operation, polygon and immediate output paths,
  certified-convex dispatch, convenience wrappers, bounded-depth behavior,
  closure, winding evidence, operand-order invariance, duplicate-input
  identities, and the absence of degenerate or exact duplicate materialized
  triangles (including independently indexed duplicates).
- `boolean_box_oracle` checks one through four boxes against an independent
  exact cell-decomposition volume oracle. It exercises general polygon,
  immediate triangle-soup, and certified-convex APIs.
- `boolean_input_validation` mutates valid closed meshes into empty,
  out-of-range, degenerate, open, and non-PWN inputs and requires the public
  APIs to return the documented error.
- `boolean_hyperreal_representations` constructs coordinates from every public
  Hyperreal `StructuralKind`: exact rational, pi-like, exp-like, sqrt-like,
  log-like, exact trigonometric, product-constant, and opaque computable. It
  applies shared symbolic translations to box pairs, verifies the representation
  selected by Hyperreal, accepts only Hypermesh's documented explicit
  certification-boundary errors, and checks every successful Boolean volume
  against an exact translation-independent oracle. Opaque-computable pairs stay
  disjoint to keep the baseline target bounded; structurally certified families
  also exercise overlaps.
- `boolean_transformations` applies transforms to neither input, the left input,
  the right input, or both inputs before the Boolean. It exhaustively selects
  Hyperlattice's identity, signed-permutation, affine-translation,
  affine-diagonal-linear, general-affine, and projective transform classes,
  including orientation-preserving and orientation-reversing representatives.
  The left and right transform classes and Hyperreal kinds are selected
  independently, giving the full class-by-class and kind-by-kind Cartesian
  products when both operands are transformed. Identity and signed
  permutations, whose coefficients are structurally fixed, are followed by a
  value-bearing translation. The target checks matrix classification,
  batch-versus-scalar point transformation, repaired winding, convex
  certification, every Boolean output API, closure, and exact-output API
  agreement. All combinations may overlap. The fuzz-only
  `fuzz-bounded-campaign` feature uses certified dyadic intervals for
  comparisons after structural identity and exact-rational tests. Disjoint
  intervals retain their certified ordering or sign, while overlap between two
  values or with zero at 512-bit refinement is treated as approximate equality.
  Symbolic Cartesian-product cases validate polygon output and its winding
  metadata without repeating the 512-bit refinements during T-junction cleanup.
  They accept explicit certification errors, including `OpenOutput` when the
  approximate campaign policy cannot certify a closed surface. Exact
  transformed cases exercise every triangulated API and require full closure.
  Default Hypermesh builds do not enable the approximate campaign policy.

Every successful Boolean result is replayed through the public polygon-closure,
certified-triangulation, and exact triangle-quality checks. Errors from
supported default-config exact inputs are treated as fuzz failures rather than
discarded.

Compile every target:

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
```

Run bounded smoke campaigns from the repository root:

```sh
cargo +nightly fuzz run polygon_predicates --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run bvh_queries --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run mesh_and_hull --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run boolean_pipeline --fuzz-dir fuzz -- -max_total_time=30 -timeout=20
cargo +nightly fuzz run boolean_box_oracle --fuzz-dir fuzz -- -max_total_time=30 -timeout=10
cargo +nightly fuzz run boolean_input_validation --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run boolean_hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30 -timeout=20
cargo +nightly fuzz run boolean_transformations --fuzz-dir fuzz -- -max_total_time=30 -timeout=20
```

The Boolean targets are intentionally separate because certified arrangement
construction is much more expensive than predicate, BVH, oracle, and
input-validation work. Run long campaigns for each target independently.
Minimize every crash and promote it to a deterministic regression test.
