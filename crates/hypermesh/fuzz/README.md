# Hypermesh fuzzing

The targets construct bounded exact-real geometry across rational and symbolic
Hyperreal representations. Primitive floats are not used to choose topology.
Every target supplies an explicit `APPROXIMATE_512` `MeshContext`, and every
successful operation exposes whether its 512-bit terminal was consumed.

The Boolean suite is split by purpose:

- `boolean_pipeline` requires supported boxes, tetrahedra, octahedra,
  subdivided surfaces, disconnected components, and variadic inputs to
  succeed through the canonical `boolean` entry point. It checks every
  operation, shared multi-expression programs, retained-native versus raw
  views, closure, winding evidence, operand-order invariance, duplicate-input
  identities, round-trip materialization, and the absence of degenerate or
  exact duplicate triangles (including independently indexed duplicates).
- `boolean_box_oracle` checks one through four boxes against an independent
  exact cell-decomposition volume oracle. It exercises raw and retained-native
  views plus single-operation and shared multi-expression programs.
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
  batch-versus-scalar point transformation, repaired winding, retained convex
  facts, canonical Boolean programs, closure, and exact volume agreement. All
  combinations may overlap. The fuzz-only
  `fuzz-bounded-campaign` feature uses certified dyadic intervals for
  comparisons after structural identity and exact-rational tests. Disjoint
  intervals retain their certified ordering or sign, while overlap between two
  values or with zero at 512-bit refinement is treated as approximate equality.
  Symbolic Cartesian-product cases validate the shared arrangement output and
  winding metadata without repeating avoidable 512-bit refinements. They
  accept only explicit certification-boundary errors. Exact transformed cases
  require full closure.
  The optional feature changes campaign bounds only; it does not select the
  predicate policy.

Every successful Boolean result is replayed through batch validation, public
closure evidence, and exact triangle-quality checks. Errors from
supported inputs whose selected policy can decide every required predicate are
treated as fuzz failures rather than discarded.

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
