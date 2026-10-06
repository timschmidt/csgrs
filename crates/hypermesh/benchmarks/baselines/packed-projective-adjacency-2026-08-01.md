# Packed projective adjacency ownership — 2026-08-01

This is Phase 7 checkpoint 27 of the workspace Hypermesh path-completeness
plan. The retained implementation is Hypermesh
`baccdc5c3a7f9174313311ec657670b8820d59f0`, based on checkpoint 26 source
`0a6607569c611cae0a973259541047720c2ad30b`. The scalar and policy base remains
Hyperreal `a90fd36aca8df4aab4661c068f2b29961d657da2` and Hyperlimit
`3e5d8816cd32bba46f48e0c6c13ab7a9da227784`.

## Outcome

The compact certified-convex input builder now stores each undirected
adjacency owner in four machine words instead of five. The omitted endpoint is
recoverable from the undirected key and the stored directed start. One
three-edge lookup pass now both selects the first exact coplanar support and
captures the complete pre-insertion occupancy mask; insertion no longer probes
the same edges again in release builds.

Across both policies, the generated 13,452-triangle row retires 0.605% fewer
instructions, 0.904% fewer branches, and uses 0.938% less task clock. The
6,144-triangle dense-axis control is effectively instruction-neutral
(-0.064%) while clock improves 2.05%. Its adjacency path is disabled, so that
row principally guards linked-layout spillover. The retained-polygon control
also excludes the changed path and stays neutral at -0.086% instructions.

The generated adjacency arena shrinks exactly 161,280 bytes, or 20%, while it
is live. Overall Heaptrack peaks remain 7.50 MiB generated, 2.34 MiB boxes,
and 11.67 MiB retained because another phase owns each rounded process peak.
Allocation counts and reconstructed temporary counts are unchanged. Canonical
linked consumers move by at most 480 file/text bytes (0.026%), and the repeated
probe's aggregate text/data/BSS shrinks four bytes.

## Exact ownership invariant

The old record was
`[other, stored_start, stored_end, owner_triangle, next]`. The retained record
is `{ other, stored_start, owner_triangle, next }`.

For a matching non-self query, the undirected key proves that the omitted
stored endpoint is the query endpoint other than `stored_start`. A query whose
start equals `stored_start` therefore has the same orientation and requests an
inverted support; the reverse query does not. For a self-edge, both old
direction tests matched and the old first branch returned non-inverted; the
new representation explicitly preserves that result. First-owner lookup order
is unchanged.

`adjacent_coplanar_support_index` still examines candidate owners in source
edge order and retains the first support whose remaining point classifies
exactly `On`. After finding it, the helper skips further exact classifications
but completes the remaining two ownership lookups. The resulting three-bit
mask is a snapshot taken before the current triangle is inserted.

That snapshot is exhaustive on every reachable insertion path:

1. adjacency storage is constructed only when the mesh is not admitted to the
   mesh-level axis cache and no supplied-plane array exists;
2. approximate axis positions exist only for a mesh admitted to that axis
   cache, so an individual axis-looking face in an adjacency mesh still enters
   the adjacency helper;
3. supplied supports are possible only when the supplied-plane array exists,
   which disables adjacency storage; and
4. all three vertex indices are bounds-checked before the helper runs.

Certified source triangles have three distinct undirected edges, so inserting
one absent edge cannot invalidate either of the other two mask bits. Existing
malformed internal owner references remain occupied and skipped, matching the
old checked insertion behavior. A new three-triangle mixed-axis regression
verifies in debug and optimized builds that an axis-looking middle face cannot
replace the first owner and that a later coplanar face reuses the original
support.

## Policy and complete fallback behavior

No `Real` equality, predicate, topology, cache, public API, or terminal policy
changes. Support reuse remains exact and all independent support validation
continues through the operation's immutable `DecisionContext`.

Under `STRICT`, an unresolved predicate remains a typed indeterminate result.
Under `APPROXIMATE_512`, only Hyperlimit's terminal 512-bit equality/sign
interpretation may resolve an otherwise unresolved predicate. The packed
record contains only source indices and orientation; it cannot consume or
invent approximation.

The compact path still requires exactly two certified-convex operands and is
excluded when retained polygons replace source triangles. Retryable compact
failures still rebuild the ordinary full polygon soup and continue through the
full projective candidate and general subdivision engine. Non-retryable index,
shape, arithmetic, and degeneracy errors propagate unchanged. No compatibility
shim was added.

## Exact output gates

| Fixture | Input triangles | Output vertices | Output triangles |
| --- | ---: | ---: | ---: |
| Generated projective | 13,452 | 154 | 304 |
| Retained arrangement | 4,524 | 625 | 1,246 |
| Dense subdivided boxes | 6,144 | 27 | 50 |

Union, intersection, difference, and symmetric difference pass under both
policies. Strict and approximate-policy meshes are exactly equal; each passes
exact directed closure and exact nondegeneracy. Polygon and immediate APIs
agree.

## Serialized CPU work

Checkpoint 26 and candidate repeated-operation executables were pinned to
logical CPU 9 in parent/candidate/candidate/parent order. Each process builds
its fixture once and repeats a complete immediate union. Retired instructions
are the deterministic retention gate; paired task clock and cycles are
corroborating measurements.

| Fixture / policy | Repetitions | Task | Cycles | Instructions | Branches |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 501 | -1.078% | -1.089% | -0.6056% | -0.9048% |
| Generated / `APPROXIMATE_512` | 501 | -0.797% | -0.810% | -0.6053% | -0.9041% |
| Dense boxes / `STRICT` | 10,001 | -1.494% | -1.515% | -0.0666% | +0.0071% |
| Dense boxes / `APPROXIMATE_512` | 10,001 | -2.587% | -2.235% | -0.0619% | +0.0165% |
| Retained / `STRICT` control | 51 | -0.922% | -0.869% | -0.0864% | -0.1269% |
| Retained / `APPROXIMATE_512` control | 51 | -1.526% | -1.442% | -0.0861% | -0.1263% |

Policy-paired movements are -0.938% task clock / -0.6055% instructions for
generated input, -2.050% / -0.0643% for boxes, and -1.226% / -0.0863% for the
retained control. The retained measurements use the same production change
before the final regression-only source addition; that path cannot execute the
compact builder and is a directional linked-layout control, not an attribution
claim. Raw brackets are in the companion TOML.

## Large-fixture heap

Heaptrack covers fixture construction plus one complete immediate union.

| Fixture | Allocations | Reconstructed temporaries | Peak heap | RSS range |
| --- | ---: | ---: | ---: | ---: |
| Generated projective | 200,756 | 10,359 | 7.50 MiB | 17.72–17.74 MiB |
| Dense boxes | 27,212 | 81 | 2.34 MiB | 10.84–10.87 MiB |
| Retained arrangement | 454,005 | 28,735 | 11.67 MiB | 20.95–20.96 MiB |

The exact-source generated and box recordings are
`/tmp/hypermesh-packed-adjacent-exact-final-{generated,boxes}-{strict,approximate-512}.zst`.
The production-identical retained recordings are
`/tmp/hypermesh-packed-adjacent-fullscan-retained-{strict,approximate-512}.gz.zst`.

The generated left operand has 13,440 triangles. Its adjacency vector reserves
`ceil(13,440 * 3 / 2) = 20,160` entries. Reducing each entry from 40 to 32 bytes
therefore changes that arena from 806,400 to 645,120 bytes: exactly 161,280
bytes and 20% less live heap. The box mesh uses the axis path and the retained
fixture excludes compact input construction, so neither owns this arena.

## Linked code and call graph

The implementation changes 51 production lines and deletes 25; focused tests
add 47 and delete six. It adds no public API or compatibility layer.

| Consumer | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| General release native text | 4,065,204 | 4,064,940 | -264 / -0.0065% |
| Immediate release native text | 4,098,868 | 4,098,588 | -280 / -0.0068% |
| General release WASM `wasm-opt -Oz` | 2,727,577 | 2,727,780 | +203 / +0.0074% |
| Immediate release WASM `wasm-opt -Oz` | 2,742,625 | 2,742,824 | +199 / +0.0073% |
| General size native text | 1,870,850 | 1,871,322 | +472 / +0.0252% |
| Immediate size native text | 1,883,310 | 1,883,790 | +480 / +0.0255% |
| General size WASM `wasm-opt -Oz` | 1,165,788 | 1,165,835 | +47 / +0.0040% |
| Immediate size WASM `wasm-opt -Oz` | 1,176,167 | 1,176,214 | +47 / +0.0040% |

The equal-work repeated executable grows 344 file bytes and 268 text bytes;
its aggregate text/data/BSS shrinks four bytes because BSS layout moves by
-272 bytes. Canonical native aggregates move +8 bytes or zero.

The comparable Hypermesh graph moves from 8,028 nodes / 19,820 edges to
8,031 / 19,827. The five-crate graph moves from 19,718 / 39,444 to 19,721 /
39,451. These small counts include the renamed internal insertion node,
receiver aliases, and regression-test calls; no policy or terminal spine was
added.

## Cycle profile

The exact-source CPU-9 frame-pointer profile covers 501 strict generated
unions, 9,331 samples, zero lost samples, and approximately 19.586 billion
cycles. Largest self owners are four-by-two signed-product summation 5.91%,
lossy rational export 4.73%, compact input construction 4.18%, six-by-two
summation 4.01%, crossing-event splitting 3.84%, exact normalization 2.80%,
mixed-width GCD 2.79%, allocator work 2.75%, word GCD 2.61%, compact
projective preparation 2.45%, and exact-rational coordinate classification
2.38%.

Compact input construction sampled at 4.90% in checkpoint 26 and 4.18% here.
Sampling attribution varies; paired retired instructions are authoritative.
The profile is `/tmp/hypermesh-packed-adjacent-exact-final.data`.

## Competitive and historical controls

One final CPU-9 Criterion session reports:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Projective generated union | 6.1450–6.2308 ms (6.1893 center) | 741.04–749.76 us (745.40) | 651.23–654.46 us (652.82) |
| Dense exact-cell boxes | 720.78–737.74 us (730.34 center) | 6.7387–6.7686 ms (6.7492) | 4.2810–4.3027 ms (4.2895) |

The generated center is 0.81% below checkpoint 26. Hypermesh remains 8.30x
slower than Boolmesh and 9.48x slower than Manifold on this exact-heavy row.
The dense-box center is 0.64% below checkpoint 26; Hypermesh is 9.24x faster
than Boolmesh and 5.87x faster than Manifold. Competitors are throughput
comparators, not exactness oracles.

The directional retained historical baseline remains 944.8 ms, 67.74 MiB,
and 5,020,891 allocations. Current direct work remains about 34.77 ms,
11.67 MiB, and 454,005 allocations: 96.32%, 82.77%, and 90.96% lower. Fixture
and implementation evolution make this a trend, not a direct A/B.

## Rejected experiments

- A three-word packed edge regressed generated instructions about 0.22%.
- A four-word early-return mask was superseded by the complete scan and grew
  linked text about 5.8 KiB.
- Passing already-fetched triangle points changed compiler layout and regressed
  dense-box instructions about 0.28%.
- Inline, cold out-of-line, and dual-mode fallback scans for a suspected
  mixed-axis path regressed the dense control. The path is structurally
  impossible because axis caching and supplied planes both exclude adjacency;
  the release regression now proves the reachable mixed-face behavior.

All losing production forms were removed.

## Validation

The final implementation passes:

- default and no-default test matrices with 1,060 library tests, and the
  all-feature matrix with 1,061;
- 1,060/1,060 default library tests under nightly AddressSanitizer;
- warning-denied all/no-default Clippy and rustdoc;
- every fuzz-bin check and all-feature benchmark compilation;
- release opt-in every-operation exactness under both policies,
  polygon/immediate agreement, 3,360/13,440-triangle stress, and complete
  11,894-triangle input validation;
- all-family dispatch tracing, with zero unknown facts and zero fallback/abort
  events on the generated compact row;
- four exact-source and two retained large-fixture Heaptrack recordings,
  serialized CPU counters, native/WASM size controls, competitive Criterion,
  the exact-source frame-pointer profile, and the five-crate call graph; and
- formatting and diff checks.

The temporary repetition hook was removed before commit. The approximately
56-minute full-resolution rotated Boolean was not rerun because it enters the
unchanged ordinary non-certified-input path; this adjacency optimization
cannot execute there.
