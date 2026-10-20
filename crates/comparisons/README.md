# exactCorelib / Hyper comparison

This harness now lives in the CSGRS monorepo. See the
[integration and reproduction guide](INTEGRATION.md) for the pinned import,
current validation, external oracle location, and historical-evidence limits.

The standalone code and evidence from the later exactCorelib, constructive-real,
and exact-real ecosystem audits are consolidated in [audits/](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/comparisons/audits/README.md).
That index documents the qualification suites, recovered experiments, source
snapshots, dependencies, and byte-for-byte migration verification.

This directory is a standalone differential-test and benchmark harness for the
exactCorelib checkout selected by `EXACTCORE_ROOT` (by default
`../../../exactCorelib-main/trunk`) and the sibling Hyper crates in this monorepo.

**No exactCorelib source code is included in this repository.** The C++ files
are independently written adapters against exactCorelib's public headers. At
build time the harness compiles a separately obtained exactCorelib checkout;
set `EXACTCORE_ROOT` to its `trunk` directory when it is not located at the
default path. See [THIRD_PARTY.md](THIRD_PARTY.md) for the dependency
and licensing boundary.

The comparison is semantic: aliases, constructors, accessors, formatting, and
crate re-exports are assigned to the operation they serve instead of being
counted as separate algorithms. Every operation for which both codebases expose
the same mathematical contract is represented in
[`coverage/comparable-api.tsv`](coverage/comparable-api.tsv). The independent
crate audit is in [`coverage/hyper-crates.tsv`](coverage/hyper-crates.tsv).
The capability survey, defect evidence, and test inventory are in the
[`exhaustive comparison report`](COMPARISON_REPORT.md). The upgraded 160-pair
native-fixture campaign, confidence intervals, fairness analysis, and every
operation result are in the
[`retained-native benchmark report`](RETAINED_BENCHMARK_REPORT.md). The original
one-shot Rust/C-ABI integration measurements remain in the
[`adapter benchmark report`](BENCHMARK_REPORT.md). Retained heap scaling,
allocation traffic, RSS/PSS, output-carrier costs, the triangle-intersection
leak, and the full 160-operation memory sweep are covered in the
[`comprehensive memory report`](MEMORY_BENCHMARK_REPORT.md); every measured row
is preserved in its
[`generated results appendix`](MEMORY_SWEEP_RESULTS.md).

## Run it

The native oracle requires a C++ compiler plus GMP and MPFR development
libraries. The build script compiles exactCorelib at `CORE_LEVEL=3`, links it
into the Rust test process, and exposes narrow C ABI calls from
`cpp/exactcore_oracle.cpp`. It also compiles the benchmark-only retained fixture
ABI in `cpp/exactcore_bench.cpp`.

```console
cargo test -- --test-threads=1
cargo test --all-features -- --test-threads=1

cargo bench --bench comparable
cargo bench --features curve --bench curves
cargo bench --features mesh --bench meshes
cargo bench --features path --bench paths
```

Memory profiling is kept behind a separate feature so allocator interposition
cannot contaminate Criterion measurements. Every tuple runs in an isolated
worker. The default campaign combines the 160 fixed-input comparable operations
with the deterministic line and triangle size sweeps:

```console
cargo build --locked --release --features memory-profile --bin memory-sweep
target/release/memory-sweep --repetitions 2 \
  --output target/memory-sweep/results.tsv
node scripts/analyze-memory.mjs target/memory-sweep/results.tsv summary
```

The default grid produces 392 worker measurements: one streaming retained-input
pair for each of the 160 comparable operations, plus 72 scaling measurements
across the three geometry workloads and two output modes. `memory-profile`
automatically enables the curve, mesh, and path fixtures needed for exhaustive
coverage. Pass `--operations-only` or `--scaling-only` to select one campaign,
`--operations` to select concrete IDs, or `--quick` for a 50-worker smoke run.
`--list-operations` prints the catalog, and `--help` lists all controls.

Each paired case now registers three functions:

- `exactCorelib`: the original one-shot adapter, including parsing, native input
  construction, FFI, and any result serialization;
- `exactCorelib-retained`: fixed native inputs constructed once, a batched C++
  operation loop, and no result serialization; and
- `hyper*`: the corresponding operation on retained Rust inputs.

The retained fixture amortizes FFI to one call per Criterion sample and uses
compiler barriers plus native result observation on every iteration. Result and
working-value construction intrinsic to the operation remains timed.

The recorded campaigns pinned the process to one logical CPU and used explicit
shorter collection settings; their exact commands and environments are in the
benchmark reports. With raw Criterion artifacts present, regenerate either
summary or full table with:

```console
node scripts/analyze-criterion.mjs target/criterion summary retained
node scripts/analyze-criterion.mjs target/criterion markdown retained
node scripts/analyze-criterion.mjs target/criterion summary adapter
```

Criterion reuses the same Hyper IDs across tiers, so a newly collected retained
campaign does not reconstruct the historical adapter report from mutable
`target/criterion` artifacts; rerun both adapter and Hyper filters together when
collecting a new adapter comparison.

The native exactCore path must be serialized. Parallel Rust test workers can
invoke independent exactCore geometry operations concurrently and have
reproducibly aborted in native deallocation with `free(): invalid size`; the
same tests pass with one worker. This concurrency hazard and its evidence are
covered in the exhaustive report.

For a quick benchmark compile/execution check without collecting statistics:

```console
cargo bench --bench comparable -- --test
cargo bench --features curve --bench curves -- --test
cargo bench --features mesh --bench meshes -- --test
cargo bench --features path --bench paths -- --test
```

Known contract defects are executable, ignored regression tests. They are left
ignored because they intentionally fail against the mathematically expected
contract; run a named test with `--ignored --exact` when investigating one.
The coplanar `Triangle3d` clipping defect is documented but deliberately not
executed because the exactCore implementation can dereference objects owned by
a destroyed temporary and terminate the process.

## What exactCorelib supports

The checkout contains substantially more than elementary predicates:

- An exact-number tower: arbitrary-precision integers and rationals, MPFR-backed
  floating values, interval and filter machinery, and lazy `Expr` real values.
  At the configured level, `Expr` includes exact algebraic arithmetic and
  certified signs alongside square/nth roots, powers, logarithms,
  exponentials, trigonometric functions, constants, floor, and ceiling.
- Algebra: generic complex numbers; univariate and bivariate polynomials;
  evaluation, arithmetic, composition, derivatives, pseudo-division,
  resultants, discriminants, GCD/square-free helpers, Sturm/Descartes-style
  root counting, and isolating intervals.
- Linear algebra: dynamically sized vectors and matrices, dot products, norms,
  wedge/cross products, matrix arithmetic, transpose, determinant, adjugate,
  and inverse support. The extensions used here also provide fixed 2D/3D/4D
  conveniences.
- Exact 2D geometry: points, lines, segments, and circles; orientation and
  signed area; collinearity/betweenness; incidence, containment, intersection,
  projection and distance; segment topology; and the incircle predicate.
- Exact 3D geometry: points, lines, segments, planes, triangles, and polygons;
  tetrahedral orientation/volume; sidedness, incidence, coplanarity,
  containment, intersection dimension, projection, and distance.
- Demonstration algorithms built on those primitives, including convex hull,
  Delaunay and constrained Delaunay triangulation, and Voronoi construction.
  For the comparable Delaunay cell contract, the harness uses an independently
  written exhaustive empty-circumcircle enumeration over exactCorelib's public
  exact scalar types; it does not copy the `dt4` demo implementation.

The Hyper workspace goes beyond this surface with Bezier/B-spline/NURBS curve
topology, mesh booleans and B-reps, paths and route constraints, SDFs, voxels,
rendering carriers, physics, packing, evolutionary search, PCB/circuit tooling,
and part knowledge graphs. Those domain APIs are not mislabeled as comparable
when exactCore only supplies a lower-level numeric or predicate dependency.

## Coverage and methodology

The default suite covers the common kernels:

| Area | exactCorelib | Hyper | Test style |
|---|---|---|---|
| Exact scalars | `BigRat`, `Expr` | `hyperreal::Rational`, `Real` | tables, domains, property tests |
| Linear algebra | `ComplexT`, `Vector`, `Matrix` | `hyperlattice` | exact examples and randomized integer matrices/vectors |
| 2D/3D predicates | geometry extensions | `hyperlimit` | signs, topology, witnesses, distances, property tests |
| Polynomial solving | `Polynomial`, `BiPoly`, Sturm helpers | `hypersolve` | identities, resultants, GCD, roots, randomized evaluation |
| Triangulation | `dt4` lower hull | `hypertri` | exhaustive small configurations and property tests |

Feature suites exercise the same exactCore oracle through higher-level public
APIs in `hypercurve`, `hypermesh`, and `hyperpath`. `hyperbrep` was audited and
has finite-line and plane operations that would be comparable. Its checkout is
compatible with the current `hypercurve` context/outcome API, but a dedicated
exactCore differential test and benchmark suite has not yet been added, so it is
recorded explicitly as comparison-pending rather than silently counted as
compared.

`tests/coverage_manifest.rs` enforces the audit mechanically. It checks that:

- every sibling `hyper*` Cargo package appears exactly once in the crate audit;
- every native `ec_*` oracle entry point is assigned to a comparison row;
- every native entry point is used by both a differential test and a paired
  benchmark;
- the 160 concrete retained benchmark IDs exactly equal the memory-operation
  catalog, with both native and Hyper fixtures constructed and executed in the
  all-feature test suite;
- every comparable row names existing test and benchmark source files; and
- divergences identify an ignored regression, while unsafe, build-blocked, and
  comparison-pending rows carry an explicit reason.

The oracle and Hyper calls run in the same process with the same integer or
rational input. Exact values are compared textually where both sides remain
rational. Algebraic/transcendental values and Euclidean distances are compared
after numeric evaluation with stated tolerances. Root-isolation comparisons
verify counts, interval ordering, containment, and witness signs rather than
requiring two valid isolators to choose identical endpoints.

Criterion reports wall-clock costs for the public operation on each side. The
one-shot exactCore tier intentionally includes the narrow C ABI's decimal
input/output conversion. The retained tier removes fixed-input parsing,
input-object construction, result serialization, and per-iteration FFI, making
it the closer kernel comparison. It is still not a cycle-level equivalence
study: output richness, lazy evaluation, caching, precision policy, and
algorithmic contracts can differ, as detailed in the retained report.

## Findings captured as regressions

The passing tests establish agreement over the manifest's matched rows. The
ignored regressions preserve the observed differences:

1. exactCore's matrix Bareiss path loses a required column-swap sign/position.
2. `Line2d::orientation` is reversed relative to the free `orientation2d`.
3. coincident 2D lines are rejected by `Line2d::intersects` after the parallel
   check, so the reported intersection dimension is disjoint.
4. circle-to-line and circle-to-circle distance APIs clamp overlap to zero,
   unlike a signed boundary-distance contract.
5. `Plane3d::isCoincident` tests the wrong parallel condition and displacement.
6. several `Triangle3d` boolean intersection functions convert dimension `-1`
   to `true`.
7. `Triangle3d::contains` omits coplanarity and accepts points beyond an edge on
   its infinite supporting line.
8. coplanar triangle clipping can return pointers owned by a temporary
   `Polygon3d`, making that path memory unsafe.
9. the univariate resultant uses a different argument/sign orientation;
   constant-polynomial resultants are also nonstandard.
10. `disc()` is a cheap proxy without the standard discriminant's sign and
    leading-coefficient normalization.
11. the exactCore Sturm path misses one root of the cubic with roots 1, 2, 3.
12. duplicate Delaunay inputs are accepted by the exactCore-backed adapter while
    `hypertri` rejects them; cocircular output is a cell complex versus a
    tie-broken triangulation.
13. exactCore accepts a zero-length `Segment2d`; `hypercurve::LineSeg2` rejects
    it as a degenerate curve.
14. a line segment wholly inside a circle has no boundary hit in `hyperpath`,
    while exactCore's signed segment/boundary metric is negative.

Two additional exactCore-only hazards found during the audit have no Hyper API
pair and are not counted as differential failures: `Polygon3d::verify()` rejects
three-vertex polygons, and `Segment2d::setOpen(bool)` writes the `directed`
member. They are noted here so the API survey does not hide them.

## License

The comparison harness is dual-licensed under either the
[MIT License](LICENSE-MIT) or the
[Apache License 2.0](LICENSE-APACHE), at your option. External dependencies,
including exactCorelib, are not included and retain their own licenses; see
[THIRD_PARTY.md](THIRD_PARTY.md).
