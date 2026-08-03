# Policy-complete Boolean fallbacks and completion audit

Date: 2026-08-03

Status: retained as Phase 9 completion checkpoint 45; every user-directed
in-scope gate passed

Revisions:

- Hypermesh implementation parent: `c4b582ac86ea710f2d1fbc91dd06f2f93d7e9eff`
- Hypermesh retained-fact/general fallback: `ee199c3aac42107f762a3a681575d93babe8916f`
- Hypermesh coincident-output index: `0ef4a5de58bb1a5edf13136bfa6f6e717daf4204`
- Hypermesh bounded finite candidate keys: `074afe11267e48aacdbfa5a4d733d14d894de3ff`
- Hyperreal explicit sign/conversion boundary: `471a0ae564108425021b3a120fb9ccd0ed532c92`
- Hyperreal bounded surd reconstruction: `6c7b8325382135e424dfdb1d7442d454c9665d75`
- Hyperlattice: `d11ca2f0e825d8e26048cfda5d1101df21dcfef0`
- Hyperlimit: `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri: `c47601266e0b9b17d0c5a0764fa22b18168ada73`
- Hypervoxel adapter lock graph: `30b396d`

The host is an AMD Ryzen 7 5800X3D running Fedora kernel
`7.0.4-100.fc43.x86_64`, Rust 1.97.0, and LLVM 22.1.6. Serialized runtime
rows are pinned to one CPU. Machine-readable values are in
`policy-complete-boolean-fallbacks-2026-08-03.toml`.

## Outcome

The three canonical Hypermesh Boolean entry points now exhaust the same
policy-aware fallback ladder. A native mesh first tries valid retained facts,
then reconstructs fresh input polygons, and finally retries through the raw
general EMBER path when a specialized or retained-fact path reports a
retryable classification, reference, depth, closure, planarization, or
point-at-infinity failure. Output materialization is inside that ladder, so a
candidate that constructs polygons but cannot certify its final mesh no longer
terminates before the complete path is tried.

This closes several representation-sensitive holes found by the all-target
sanitizer corpus:

- clipping invalidates both retained vertex values and construction identities;
- noncoplanar polygon intersections deduplicate numerically equal `Real` points
  through the selected predicate policy, not only by representation equality;
- native retained-polygon failure retries with newly constructed polygons;
- every retryable native/specialized failure can fall through to the raw
  general path; and
- a compact projective candidate with coincident emitted faces declines before
  acceptance, allowing the complete general path to resolve the arrangement.

The coincident-face guard is indexed without weakening equality. Retained exact
rational vertices receive an order-independent key made from their
deterministic finite binary64 views. Different keys prove that exact-equal
vertex sets are impossible; matching rounded collisions are compared by the
complete policy-aware exact cycle predicate. Symbolic, non-rational,
non-finite, and unkeyed cycles remain on the complete all-pairs route. Exact
`1 + 2^-60` versus `1` is a regression case: both round to the same key and are
still proved unequal exactly.

Hyperreal also removes the public `Computable::sign` query whose `NoSign`
result conflated exact zero and exhausted refinement. All owned callers use
the explicit `sign_until` result; no deprecated forwarding method or
compatibility shim remains. Owned `Real` primitive conversions now use a
cache-independent bounded, exact-first rounding schedule; their terminal
primitive value remains explicitly lossy and non-certifying, like borrowed
scheduling/rendering views. An oversized exact
quadratic surd is retained as one opaque computable instead of recursively
reconstructing the same `sqrt` expression after the bounded rational shortcut
declines.

No new dependency, feature, public mesh API, mutable global policy, or
compatibility path was introduced.

## Policy and exactness contract

Every public topology operation receives an immutable one-byte
`MeshContext`. `DecisionContext` forwards exactly its selected
`PredicatePolicy` into Hyperlimit and aggregates any consumed terminal
certainty into `MeshOutcome`.

- `STRICT` accepts only exact, filtered, or certified-refinement outcomes. An
  unresolved predicate returns a typed `PredicateUndecided` or a more specific
  operation error.
- `APPROXIMATE_512` follows the identical certified cascade. Only after it is
  exhausted may Hyperlimit interpret equality/sign at the terminal 512-bit
  boundary, and consumption changes the aggregate result to
  `Approximate512Consumed`.
- Binary floating values are never topology truth. They can propose buckets,
  work order, or conservative rejection only where exact equality preserves
  the key. Every collision or inconclusive filter reaches exact/policy-aware
  comparison.
- Failed attempts reuse the same operation context, so terminal certainty can
  only be conservatively retained across retries; it cannot be erased.
- Reusable mesh facts are either certified or tagged approximate and can only
  be consumed by a policy that permits their certainty. A later strict proof
  upgrades, rather than aliases, an approximate fact.

Production searches find no hard-coded mesh policy, policy-free topology
wrapper, deprecated API, compatibility shim, unchecked winding transition, or
hidden default subdivision cap. `DEFAULT_MAX_DEPTH` is `usize::MAX`; an
explicit caller bound returns `SubdivisionDepthLimit` rather than accepting
partial output. Bounded samples and small direct-search limits select work
order or an accelerator only; every decline retains an unbounded-by-that-
constant exact fallback.

## Complete output/fallback ladder

For certified two-convex input, Hypermesh can attempt the compact projective
engine. Acceptance requires all of the following:

1. exact input/PWN and convex facts are valid for the current policy;
2. point, plane, edge, winding, and face identities survive exact validation;
3. no selected output polygons coincide;
4. the selected construction-aware, scan, or recovery triangulation succeeds;
5. every accepted triangulation is exactly closure-certified; and
6. native materialization is exactly nondegenerate and valid.

Any retryable failure proceeds through the remaining construction recovery,
scan triangulation, fresh-polygon projective attempt, and raw general EMBER
routes as applicable. Non-retryable invalid input, capacity overflow, winding
overflow, and arithmetic failures remain errors; they are not hidden by a
fallback.

The general route processes finite subdivision work until leaf proof succeeds
or the caller's explicit resource bound is reached. T-junction repair and
crossing batches run to a fixed point, and every accepted output is checked for
boundary, unbalanced, and non-manifold edges.

## Correctness and path verification

The current five-crate matrix completed successfully:

- default, no-default/minimal, all-feature, and feature-combination suites;
- Hypermesh's complete 1,069-test library/integration configuration, including
  both policies in one binary;
- all-target/all-feature Clippy with warnings denied;
- all-feature rustdoc with warnings denied and formatting checks;
- every fuzz target in Hyperreal, Hyperlattice, Hyperlimit, Hypertri, and
  Hypermesh; and
- all-target benchmark and fuzz-manifest compilation.

Nightly AddressSanitizer campaigns completed without an error. The final
Hypermesh Boolean-pipeline campaign covered 1,141 corpus inputs and 1,588
executions over 31 seconds. Supporting campaigns covered input validation
(43,684 executions), exact boxes (1,956), transformations (the full 655-input
corpus), and Hyperreal representations (295). LeakSanitizer was disabled
because sandbox ptrace restrictions prevent its normal process inspection;
address and undefined memory behavior remained instrumented.

Opt-in YeahRight semantics pass for all four operations and both policies:

- every output is exact-boundaryless and contains no exact degenerate triangle;
- `STRICT` and `APPROXIMATE_512` produce identical certified results;
- polygon-result and immediate-mesh APIs agree in volume and surface area;
- the 11,894-triangle source validates as a closed PWN; and
- 3,360- and 13,440-triangle stress unions remain closed and certified.

The retained fixture has SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.

## Large-fixture heap and RSS

Final-source Heaptrack recordings include fixture construction plus one union.
The generated and retained rows exercise the projective/general output spine;
the dense row exercises 3,072 triangles per operand and the exact box family.

| Fixture / policy | Input triangles | Output V/T | Allocations | Temporary | Peak heap | Profiler RSS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 13,452 | 154 / 304 | 200,429 | 10,364 | 7.50 MiB | 18.20 MiB |
| Generated / `APPROXIMATE_512` | 13,452 | 154 / 304 | 200,429 | 10,364 | 7.50 MiB | 18.41 MiB |
| Retained / `STRICT` | 4,524 | 625 / 1,246 | 452,724 | 28,731 | 11.60 MiB | 21.59 MiB |
| Retained / `APPROXIMATE_512` | 4,524 | 625 / 1,246 | 452,724 | 28,731 | 11.60 MiB | 21.43 MiB |
| Dense boxes / `STRICT` | 6,144 | 27 / 50 | 2,138 | 65 | 1.14 MiB | 9.63 MiB |
| Dense boxes / `APPROXIMATE_512` | 6,144 | 27 / 50 | 2,138 | 65 | 1.14 MiB | 9.73 MiB |

All six outcomes are `Certified`. The complete output guard costs only one,
two, and two allocation calls on generated, retained, and dense rows versus
checkpoint 44, with no movement in the rounded useful peak.

Independent `/usr/bin/time -v` runs report 12,988/12,992 KiB maximum RSS for
generated strict/approximate, 17,664/17,740 KiB for retained, and 6,440/6,412
KiB for dense. Their one-operation elapsed times are 0.07/0.06 s,
0.04/0.04 s, and below 0.01 s respectively.

The retained historical control was 944.8 ms, 67.74 MiB heap, 5,020,891
allocations, and 82.5 MiB RSS. The current direct row is directionally 95.77%
faster (23.62x), uses 82.88% less peak heap, makes 90.98% fewer allocations,
and uses 79.09% less RSS. Historical and current output representations differ,
so these are implementation-evolution trends rather than revision A/B claims.

## Full-resolution hard gate

The committed current source passed the 11,894-by-11,894 rotated-intersection
hard gate:

| Measure | Current source |
| --- | ---: |
| Test harness | 3,312.65 s |
| Wall time | 55:12.66 / 3,312.66 s |
| User / system time | 3,295.24 s / 1.72 s |
| Maximum RSS | 329,352 KiB / 321.63 MiB |
| Major / minor page faults | 0 / 900,394 |
| Swaps | 0 |
| Outcome | `Certified`, 0 vertices / 0 triangles |

The test selected `APPROXIMATE_512`, but the aggregate remained `Certified`:
no terminal 512-bit interpretation was consumed, so every decision made by
this run is also admissible under `STRICT`. This is not relabeled as a
separately timed strict run.

This closes the old final-source caveat: the measurement is from the committed
192-bit dynamic-entry selector, not the earlier conservative 512-bit candidate.
It is 1.29% faster by harness time and 1.32% faster by wall time than that
candidate's 3,356.02/3,357.09-second measurement. RSS is 0.80% higher, inside
the same bounded 319--322 MiB class; minor faults fall 40.36%, with zero major
faults and zero swaps. Against the original approximately 116 GiB failure,
current maximum RSS is approximately 99.73% lower.

The established exact CGAL EPECK oracle for the identical rotated
intersection reports a valid empty 0-vertex/0-face output in 0.09 s at
15,516 KiB RSS. Hypermesh now matches its topology and completes with explicit
certified policy evidence, but remains approximately 36,807x slower and uses
21.23x its RSS on this adversarial full-resolution case. That is the largest
remaining exact-competitor gap and is not concealed by the strong improvement
over Hypermesh's historical failure.

## Competitive and historical runtime

Criterion competitors and Hypermesh ran in the same CPU-15 session. Background
package activity made absolute centers slower than the isolated Hypermesh-only
session, so same-session ratios are authoritative.

### Standard projective YeahRight control

| Operation | Hypermesh | Boolmesh | Manifold-rust | Hypermesh relation |
| --- | ---: | ---: | ---: | --- |
| Union | 7.2918 ms `[7.2288, 7.3539]` | 769.03 us | 677.44 us | 9.482x / 10.764x slower |
| Intersection | 5.2006 ms `[5.1985, 5.2047]` | 745.29 us | 675.06 us | 6.978x / 7.704x slower |
| Difference | 5.1610 ms `[5.1297, 5.2143]` | 770.18 us | 681.36 us | 6.701x / 7.575x slower |

The clean Hypermesh-only centers are 6.3609, 4.5865, and 4.1955 ms. The
current same-session projective union is 44.59% below the 13.1596 ms
2026-07-31 control, but the non-exact competitors retain a substantial
standard-projective throughput advantage.

### Dense 3,072-triangle-per-operand control

| Operation | Hypermesh | Boolmesh | Manifold-rust | Hypermesh relation |
| --- | ---: | ---: | ---: | --- |
| Union | 706.69 us | 7.7039 ms | 4.8766 ms | 10.901x / 6.901x faster |
| Intersection | 510.90 us | 3.9261 ms | 3.4194 ms | 7.685x / 6.693x faster |
| Difference | 645.49 us | 6.6281 ms | 4.0420 ms | 10.268x / 6.262x faster |

The immediate API remains faster than polygon materialization on the standard
projective fixture: 7.3526 versus 8.0011 ms for union, 5.2370 versus 5.8823 ms
for intersection, and 5.0926 versus 5.9080 ms for difference.

The first correct coincident-output guard used an all-pairs exact scan. Its
allocation-heavy exact comparisons made the hard projective rows unsuitable.
The modular candidate index restored those costs; the final finite-view key
then improved isolated union/intersection/difference another 0.81%, 3.27%, and
2.84% respectively. A bounds-only guard was removed completely after
regressing union 18.9%, intersection 83.7%, and difference 53.6%.

## Final cycle profile

The post-hard-gate CPU-15 profile samples the current standard projective union
at 1,999 Hz. It captured 14,537 `cycles:u` samples with zero loss and an
approximate 29.676-billion-cycle event count. The profiled Criterion center is
6.4945 ms `[6.4508, 6.5504]`.

| Self owner | Samples |
| --- | ---: |
| Mixed-width rational GCD | 4.10% |
| glibc `_int_malloc` | 4.01% |
| Exact edge-crossing splitting | 3.85% |
| Rational word GCD | 3.78% |
| Certified rational line-sign filter | 3.03% |
| Projective convex-face construction | 2.49% |
| glibc `malloc` | 2.12% |

The new coincident-output guard has no self symbol at the 0.5% reporting
threshold. Remaining cost is distributed across exact normalization,
allocation, certified crossing predicates, and supported complete geometry
strategies; each leading family has a retained optimization or measured
rejected alternative in the Phase 7 ledger. The profile is
`/tmp/hypermesh-final-v3-union.perf.data`.

## Native and WASM size

Absolute canonical consumers select the operation at runtime and materialize
the result. `release` prioritizes speed; `size` uses the repository's size
profile. WASM values are after `wasm-opt -Oz`.

| Features / consumer | Native file | Native text | Optimized WASM |
| --- | ---: | ---: | ---: |
| Default general release | 4,744,368 B | 4,080,804 B | 2,749,974 B |
| Default immediate release | 4,778,912 B | 4,113,828 B | 2,764,531 B |
| Default general size | 2,124,032 B | 1,879,786 B | 1,173,451 B |
| Default immediate size | 2,136,352 B | 1,891,558 B | 1,183,597 B |
| All-feature general release | 4,904,560 B | 4,236,113 B | 2,823,407 B |
| All-feature immediate release | 4,939,424 B | 4,269,465 B | 2,838,310 B |
| All-feature general size | 2,123,984 B | 1,879,690 B | 1,204,250 B |
| All-feature immediate size | 2,136,320 B | 1,891,478 B | 1,214,218 B |

The path-completeness guard adds 0.306--0.499% to native text or optimized
WASM relative to checkpoint 44. Against the lower-cost modular guard, final
movement is mixed and below 0.035%: release native text is 240--448 bytes
smaller, release optimized WASM is 955 bytes larger, and size-profile native
text/optimized WASM grow 176--192/289 bytes. This bounded size trade is
retained because omitting the guard allows an invalid specialized output to
avoid the complete route.

## Source and bloat review

Current `tokei` Rust code and source-directory bytes are:

| Crate | Rust code lines | `src` bytes |
| --- | ---: | ---: |
| Hyperreal | 46,365 | 2,155,294 |
| Hyperlattice | 14,725 | 662,090 |
| Hyperlimit | 17,923 | 762,752 |
| Hypertri | 7,784 | 312,015 |
| Hypermesh | 80,706 | 2,939,886 |
| Total | 167,503 | 6,832,037 |

The 2026-07-30 baseline was 150,823 lines and 6,257,423 bytes. Growth is
concentrated in exactness, policy, generative, and path-regression tests;
Hyperlimit's source bytes slightly shrink. The final Hypermesh implementation
delta from checkpoint 44 is 501 insertions and 66 deletions across production,
fuzz input validation, and the example caller. It replaces silent exits and
adds one shared retry spine rather than retaining old and new APIs.

`cargo bloat` attributes 2.0 MiB (55.3% of analyzed text) to Hypermesh,
829.7 KiB (22.5%) to Hyperreal, 526.6 KiB (14.3%) to `std`, 106.7 KiB (2.9%)
to Hyperlimit, 102.4 KiB (2.8%) to `num_bigint`, 41.5 KiB (1.1%) to Hypertri,
and 12.9 KiB (0.3%) to Hyperlattice. The largest Hypermesh symbols are the
supported complete strategies: reachability probing (49.3 KiB), projective
convex-face construction (42.8 KiB), adjacent-normal search (37.1 KiB),
support-reference tracing (36.4 KiB), support-plane reference search
(36.2 KiB), leaf processing (34.1 KiB), replacement probing (30.7 KiB), and
new-reference construction (29.8 KiB). Removing one would remove a distinct
complete fallback, not merely duplicated compatibility code.

## Dispatch, call graph, and API inventory

The final generated-union dispatch run reports 97,711 events: 676 predicates,
1,411 linear-algebra events, 6,836 object facts, 953 scalar facts, 188 detailed
facts, 48,676 exact-rational kind observations, 12,261 sign/zero observations,
94,117 exact reducer events, 12,376 approximation events, 6,341 cache events,
12,775 rational temporaries, 725 reductions, and 2,389 GCD events. Unknown
facts and fallback/abort trace events are both zero, while the output remains
154 vertices / 304 triangles and `Certified`.

The production five-crate call graph contains 19,859 nodes and 39,734 edges,
SHA-256
`74f8b8450f5f0870a8fe7388b823474aaa245c0696510fa29aad86e2263d1c04`.
Hypermesh accounts for 8,113 nodes / 20,013 edges, SHA-256
`58efe73a0004d5e4783835b92c83622ef57b39aa7cf45a6228f72d66e318651a`.
The all-source evidence graph contains 26,253 nodes / 49,630 edges, SHA-256
`4a38cc0ebaedd5067b5907dba688ad4ed3f51b875d52514692e2bd9d116ea837`.

Nightly rustdoc inventory sees 325 public Hypermesh items and 148 callables.
Static evidence correlates 33/85 callables to qualified/heuristic tests, 8/44
to benchmarks, 9/63 to fuzz targets, and 10/42 to dispatch trace; 6/23 have all
four. These counts deliberately remain an audit aid rather than a coverage
claim: receiver and re-export syntax undercounts actual execution, and trivial
carrier/accessor methods are not separate executable mesh-operation families.
The retained operation families—Boolean polygon/immediate/native,
construction/triangulation, subdivision/trace, hull, validation, transform,
intersection/query, and conversion—are represented by semantic tests and the
runtime graph. The JSON and Markdown inventory hashes are respectively
`e1fec6602e28ede2cd67d92c0e553752356b370100f8d097c23d5608519e46f2`
and `95ab7b1b9d736cf922387978d15db8cb4bdb89fdbf79f7166e6b1fa81ba1fda5`.

## Scope and completion audit

Hypercurve and HyperSolve are under concurrent development and are explicitly
excluded from this checkpoint at the user's direction. CSGRS, Hypercircuit,
HyperDRC, Hyperphysics, and Synaps-CAD transitively enter that concurrent
migration and are not modified or used as completion evidence here. Their
working trees are preserved. Hypermesh's nested UI passed all 8 tests.
Hypervoxel's independent optional Hypermesh adapter passed its complete
adapter-enabled package suite, then passed again under `--locked`; commit
`30b396d` adds only the missing Hypertri lock-graph entry. Hypervoxel's
pre-existing `benches/grid_frame.rs` edit is untouched.

The five owning crates are clean except Hyperlimit's pre-existing untracked
`hyperlimit` executable and fuzz corpus/artifacts, which are deliberately
untouched. No measurement hook remains. Exact temporary artifacts are recorded
under `/tmp/hypermesh-final-v3-*`; their durable derived values are captured
here.

## Reproduction

Representative commands are:

```sh
cargo test --locked --all-features
cargo test --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo fmt --all -- --check
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
YEAHRIGHT_BENCH=1 cargo bench --locked --bench dispatch_trace --features dispatch-trace
YEAHRIGHT_BENCH=1 cargo bench --locked --bench competitive
```

The root plan records the exact five-crate matrix, sanitizer commands,
Heaptrack artifacts, size consumers, call-graph utility invocation, and final
requirement-to-evidence mapping.
