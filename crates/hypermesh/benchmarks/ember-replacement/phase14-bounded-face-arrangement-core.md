# Phase 14 checkpoint: bounded exact face-arrangement core

Date: 2026-08-03

Implementation: `716116aade5564e3fab629f273e9999d47e439df`

Corpus follow-up: `eb5f7424bc47e6efe0127806b1b86da75efe5f68`

Status: retained, test-gated Phase 14 core. This is not the production
orchestrator, a Phase 14 exit, an EMBER cutover, radial/cell classification, or
CGAL parity. `surface_arrangement` is compiled only for tests, so Hypermesh does
not ship two Boolean engines while this boundary is hardened.

## Decision

Retain the exact bounded source-face arrangement and its direct consumption of
the canonical pairwise-intersection graph. It demonstrates the intended
replacement architecture without routing a production caller away from EMBER
before radial topology and cell classification exist.

Each convex source face is its own exact bound. The core gathers its source
cycle, transverse segments, isolated contacts, and positive-area coplanar
overlays into one planar straight-line graph. It exhausts proper crossings,
collinear overlaps, coincident edges, and T-junctions, then sends only that
face's local points and split constraints to Hypertri's convex-hull PSLG entry
point. It never constructs a whole-plane or whole-output convex hull.

## Exact representation and policy

- One operation-local `Point3` arena is paired with canonical construction
  recipes and a structural recipe index. Exact-rational contradictions for one
  recipe are typed failures.
- Distinct recipes reach the existing policy-aware point interner. `STRICT`
  preserves an undecidable equality as `PredicateUndecided`;
  `APPROXIMATE_512` can terminate only through Hyperlimit's 512-bit policy and
  updates aggregate `MeshCertainty`.
- Source vertices, source-edge/plane intersections, sorted plane triples, and
  coplanar source-edge intersections are represented structurally. Shared and
  coincident faces reuse the same global point IDs.
- Positive-area coplanar overlaps are clipped exactly in projective space while
  retaining the construction identity of every boundary line. The same overlay
  constraints are appended to both faces once.
- All orientation, point-on-segment, crossing, support-membership, and sorting
  decisions use the caller's `DecisionContext`. Hypertri receives the same
  runtime policy and its certainty is absorbed into the Hypermesh operation.
- Lifted 2-D crossings are backvalidated against the exact 3-D support plane.
  Every output triangle is proved nondegenerate and oriented like its source,
  and every authored split constraint must be present in the triangulation.

No compatibility shim, alternate scalar, epsilon, snap rounding, finite pass
limit, or hidden approximate predicate is introduced.

## Permanent structural corpus at this checkpoint

Eight focused tests cover:

1. two disconnected closed loops nested in one bounded face under both
   policies;
2. partial coplanar overlap whose six-vertex shared cell has byte-equivalent
   canonical triangle IDs on both source faces;
3. fully coincident, opposite-winding faces sharing one three-point arena and
   one unoriented triangulation;
4. crossing transverse constraints producing one canonical plane-triple point;
5. collinear overlap plus two T-junctions collapsing to one ten-edge split set;
6. an isolated point contact retained as an interior triangulation vertex;
7. a 17-by-17 grid with 289 proper crossings, 68 authored endpoints, 360 exact
   points, and 649 final constraints under both policies; and
8. a symbolic `pi + e` versus `e + pi` alias proving strict indeterminacy and
   approximate-512 terminal consumption.

The 289-crossing row exceeds the former 256-pass boundary and completes from a
finite event set. The complete all-feature matrix passed 1,098 unit tests plus
134 integration tests, with seven expected opt-in/manual ignores.

## Directional runtime and local heap comparison

The fixed release test binary was pinned to CPU 11. Callgrind 3.27.0 measured:

| Row | Work | Instructions |
| --- | --- | ---: |
| New bounded arrangement | Both policies; crossing discovery, splitting, local CDT, and exact backvalidation | 333,927,495 |
| Retained output crossing machinery | Approximate policy; discovery and split of 34 skinny triangle faces | 369,096,924 |

The new row retires 35,169,429 fewer instructions (-9.5285%) despite performing
both policy runs and complete triangulation. This is directional replacement
evidence, not an A/B speed claim: the new row begins from 34 authored face
constraints, while the retained row begins from 34 skinny triangles and 102
triangle edges.

Hypertri's two spatial Delaunay calls account for 119,288,317 inclusive
instructions (35.72%); incremental insertion itself accounts for 58,289,608
self instructions (17.46%). Those are the first Phase 17 optimization targets
after production integration fixes the final workload shape.

Massif used `--stacks=yes` on the same rows:

| Row | Useful heap maximum | Total maximum |
| --- | ---: | ---: |
| New bounded arrangement | 631,076 | 659,208 |
| Retained output crossing machinery | 1,683,275 | 1,814,944 |

Useful peak is lower by 1,052,199 bytes (-62.5090%); total is lower by
1,155,736 bytes (-63.6789%). Heaptrack independently reports 43,292 versus
82,948 allocations (-47.8083%), 11,245 versus 12,851 temporary allocations
(-12.4971%), 705.15 KiB versus 1.76 MiB peak heap, and 11.77 versus 12.81 MiB
peak RSS. Test-harness globals explain the reported end-of-process leaks and
are present in both rows.

## Required large-mesh heap guard

The canonical 6,144-triangle fixtures were measured directly under both
policies. The replacement core is still test-gated, so these rows prove no
staging regression in the current production path; they do not estimate the
future arrangement engine from a small proxy.

| Path | Policy | Useful maximum | Total maximum | Output vertices/triangles |
| --- | --- | ---: | ---: | ---: |
| General | `STRICT` | 28,730,032 | 29,956,456 | 2,410 / 4,816 |
| General | `APPROXIMATE_512` | 28,730,032 | 29,956,584 | 2,410 / 4,816 |
| Certified | `STRICT` | 1,063,718 | 1,064,976 | 27 / 50 |
| Certified | `APPROXIMATE_512` | 1,063,718 | 1,064,976 | 27 / 50 |

Useful maxima are byte-identical to the pre-Phase-14 checkpoint. Both policies
remain `Certified` and topology is unchanged.

## Source, linked size, and call graph

The arrangement implementation and proofs add 1,667 lines in a test-gated
module. The only linked production residue before cutover is the typed surface
arrangement error path. Against the immediately preceding recorded size rows:

| Consumer/profile | Native text delta | Optimized WASM delta |
| --- | ---: | ---: |
| General release | +48 (+0.0012%) | +253 (+0.0091%) |
| Immediate release | +56 (+0.0013%) | +254 (+0.0091%) |
| General size | +80 (+0.0042%) | +64 (+0.0054%) |
| Immediate size | +96 (+0.0050%) | +74 (+0.0062%) |

The workspace call-graph utility reports 20,543 nodes/40,822 edges for the five
selected crate source graph and 27,000/50,839 with tests, benches, examples,
and fuzz targets. The scanner sees 318 arrangement nodes even though rustc
removes the cfg(test) module from production. It also inventories 4,437 nodes
under `subdivision`, `segment_trace`, and `local_bsp`; that is the explicit
Phase 16 deletion surface, not retained architecture.

The graph shows direct arrangement edges to Hyperlimit's proper-intersection
construction and to Hypertri's `Constraint`, `ExactPoint`, runtime context, and
`constrained_delaunay_convex_hull`; there is no arrangement-to-EMBER edge.

## Historical and competitive gauge

The established favorable common-contract row remains Hypermesh exact-box
union at 8.1114 us versus Boolmesh 73.329 us, Manifold-rust 60.191 us, and CGAL
6.0.3 EPECK 156.159 us. The governing adversarial deficit also remains open:
the historical full-resolution Hypermesh result is 3,312.66 seconds versus
approximately 0.09 seconds for CGAL EPECK, with 21.23x higher RSS. This
test-gated checkpoint neither reruns that multi-hour EMBER path nor claims that
the new engine has closed it. Phase 17 retains the per-case CGAL parity gate.

## Validation and remaining work

The focused and full all-feature suites, formatting, warnings-denied Clippy,
and warnings-denied rustdoc pass. Hypertri's complete all-feature matrix and
30-second ASan/libFuzzer topology campaign passed at its prerequisite
checkpoint.

Phase 14 still requires production ownership, compacting the ordered maps and
temporary scans after real workload profiles, and complete shared-face exit
evidence. Phase 15 must build exact radial rings, volumetric cells, and winding
truth propagation. Phase 16 must atomically migrate callers and delete EMBER;
until then no compatibility or dual-engine route will be added.

## Reproduction

```sh
cargo test --locked surface_arrangement --lib
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo test --locked --release --no-run --lib
taskset -c 11 valgrind --tool=callgrind \
  --callgrind-out-file=/tmp/hypermesh-phase14-dense.callgrind \
  target/release/deps/hypermesh-<hash> \
  surface_arrangement::tests::dense_crossing_discovery_exhausts_the_finite_grid_without_a_pass_limit --exact
taskset -c 11 valgrind --tool=massif --stacks=yes \
  --massif-out-file=/tmp/hypermesh-phase14-dense.massif \
  target/release/deps/hypermesh-<hash> \
  surface_arrangement::tests::dense_crossing_discovery_exhausts_the_finite_grid_without_a_pass_limit --exact
cargo build --locked --release --example large_mesh_heap_probe
taskset -c 11 valgrind --tool=massif --time-unit=B --detailed-freq=1 \
  --massif-out-file=/tmp/hypermesh-phase14-large-general-strict.massif \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
cargo run --manifest-path ../tools/hyper-callgraph/Cargo.toml --release -- \
  --root .. --out-dir /tmp/hypermesh-phase14-callgraph \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh --format json
```
