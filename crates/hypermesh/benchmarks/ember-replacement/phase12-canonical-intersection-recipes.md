# Phase 12/13 checkpoint: canonical retained intersection recipes

Date: 2026-08-03

Implementation: `61dab4dd44bd146609a1e1158897c34de66ccd31`

Source parent: `43f9c85385f34ca541e0afc554e69d6ae316a3fb`
(the intervening `96c4619d` commit contains evidence only)

Status: retained Phase 12–13 substrate checkpoint. This is not completion of
Phase 12, Phase 13, the arrangement engine, EMBER removal, or CGAL parity.

## Decision

Retain the canonical construction recipes, proof-keyed symbolic interning,
exact-rational alias canonicalization, provenance-aware pairwise cache, and
operation-local plane remapping.

The 6,144-triangle general fixture adds 44,935,312 deterministic instructions
(0.1968%), 6,731,151 modeled branches (0.1970%), and 3,608 useful peak heap
bytes (0.0126%) versus the exact frozen source parent. The certified large path
has byte-identical useful heap. Canonical consumer growth is 0.172–0.436% native
text and 0.341–0.526% after `wasm-opt -Oz`. The metadata is required for exact
shared corefinement, performance has priority over size, and the costs are
bounded, so the checkpoint is accepted provisionally. Phase 16 still owns the
much larger historical-engine deletion and Phase 17 owns recovery of avoidable
runtime, heap, and linked-size debt.

## Canonical recipe model

Every point in the retained pairwise graph now has one aligned optional
`ConstructionVertexIdentity`. The existing affine `Point3` arena remains the
single materialization cache needed by current consumers; there is no second
coordinate arena.

For normalized source faces:

- an input vertex retains its `Source` identity;
- a source edge crossing another face's support becomes `SourceEdgePlane`;
- an already split edge crossing a support becomes a sorted `PlaneTriple`; and
- exact numeric aliases retain the lexicographically smallest recipe, making
  the result independent of discovery and operand order.

Face support IDs use an operation-local namespace (`mesh = u32::MAX`) and the
checked face index. Polygon-order cache remapping rewrites only those local
support IDs, preserves persistent source/projective plane IDs, and re-sorts
plane triples. Invalid or non-bijective remaps remain typed failures.

The narrow-phase public API is unchanged. Public `intersect_polygons` calls do
not clone or retain construction recipes; the internal streamed graph path
requests them. This is one classifier and one graph builder, not a compatibility
adapter or a second Boolean engine.

## Proof-safe interning and contradiction handling

The builder hashes recipes with the existing compact storage hasher and stores
one fingerprint head plus a checked alias chain. A fingerprint is scheduling
only: every match compares the complete `ConstructionVertexIdentity`. A unit
test injects a false fingerprint-head collision and proves that distinct
recipes remain distinct.

A repeated structural recipe is an incidence proof, so it can share a symbolic
point without asking Hyperlimit to decide equality. If two materializations for
one recipe both expose exact-rational coordinates and disagree, construction
fails with `UnknownClassification`; it never selects one silently. Exact
numeric aliases with different recipes use the existing exact-only point
interner, and reverse insertion orders produce the same canonical recipe.

The fingerprint map and alias chain are builder-only. Finalization drops both;
the retained graph pays only one optional compact recipe per materialized
point. Points with no recipe do not reserve hash/alias capacity.

## Cache provenance and policy isolation

Geometrically equal polygon families are no longer sufficient for pairwise
cache reuse. Exact-order and permuted-order hits also require identical retained
construction cycles. The new regression constructs identical triangles with
different source edge IDs and proves that they receive distinct graph results.

The cache is operation-local. It cannot move an `APPROXIMATE_512` conclusion
into a later `STRICT` operation. Required geometry comparisons still run
through the operation's `DecisionContext`:

- `STRICT` declines when exact/certified evaluation is undecided;
- `APPROXIMATE_512` may terminate only in Hyperlimit's 512-bit evaluation;
- terminal consumption remains aggregated in `MeshOutcome` certainty; and
- structural recipe reuse and exact-rational aliasing consume no terminal
  policy decision.

Graph packing, recipe remapping, and finalization perform no scalar predicate.
The permanent symbolic contact and full policy-order corpus from the preceding
checkpoint continue to pass.

## Complete paths exercised at this boundary

Focused tests cover:

- transverse source-edge/support crossings and retained recipes;
- source-edge-plane and plane-triple order remapping;
- symbolic structural reuse without equality;
- an injected fingerprint collision with full-identity disambiguation;
- rejection of contradictory exact materializations;
- exact numeric aliases in both insertion orders;
- cache rejection for equal geometry with different provenance; and
- every previously admitted disjoint, point, segment, and coplanar-area class.

The ASan/libFuzzer polygon-predicate campaign loaded 516 seeds and completed
37,703 executions with no failure. Default, no-default, and all-feature test
matrices passed 1,222, 1,222, and 1,224 tests respectively, with seven expected
opt-in ignores in each matrix.

## Large-fixture topology and heap

Both policies preserve certified topology:

| Probe | Input triangles | Output vertices | Output triangles | Certainty |
| --- | ---: | ---: | ---: | --- |
| `boxes-3072-general` | 6,144 | 2,410 | 4,816 | `Certified` |
| `boxes-3072` | 6,144 | 27 | 50 | `Certified` |

Direct Massif maxima (`--time-unit=B --detailed-freq=1`) are:

| Path/policy | Parent useful | Candidate useful | Useful delta | Candidate total |
| --- | ---: | ---: | ---: | ---: |
| General strict | 28,726,424 | 28,730,032 | +3,608 (+0.0126%) | 29,955,736 |
| General approximate-512 | 28,726,424 | 28,730,032 | +3,608 (+0.0126%) | 29,956,664 |
| Certified strict | 1,063,718 | 1,063,718 | 0 | 1,064,976 |
| Certified approximate-512 | 1,063,718 | 1,063,718 | 0 | 1,064,976 |

Total-byte deltas are +2,504 strict and +3,384 approximate-512. Allocator
bookkeeping explains the small policy difference; useful bytes are identical.

## Deterministic and native performance

The frozen parent SHA-256 is
`853caa6c409adb8b5b125ab609044a7394d2c3dad1bb28dce07906f530315241`;
the candidate is
`5fe2b5555e96d02cf66c59d5743375e64f36bcbdf216ab5e17c3110b826eee6c`.

| General strict Callgrind event | Parent | Candidate | Delta |
| --- | ---: | ---: | ---: |
| Instructions | 22,834,571,470 | 22,879,506,782 | +0.1968% |
| Total branches | 3,416,018,761 | 3,422,749,912 | +0.1970% |
| Mispredictions | 103,193,291 | 103,568,749 | +0.3638% |

Two reverse-bracketed ten-process `perf stat` batches on CPU 11 measured a
0.5715% lower mean task clock and 0.8037% lower mean cycles for the candidate,
while hardware instructions grew 0.2658%, branches 0.8230%, and branch misses
3.2402%. Frequency, hash layout, and branch-miss noise make those elapsed
results unsuitable for a speed claim. Acceptance is based on required exact
functionality and the deterministic bounded-work result, not the favorable
clock sample.

## Historical and competitive gauge

The specialized exact-cell path did not change, so expensive competitors were
not rerun for this carrier checkpoint. The pinned shared-contract context
remains 8.1114 us for Hypermesh exact-box union, 73.329 us for Boolmesh, 60.191
us for Manifold-rust, and 156.159 us for CGAL 6.0.3 EPECK. These separately
compiled favorable-case numbers are context, not a universal ratio.

The adversarial full-resolution result remains the governing deficit:
Hypermesh's historical 3,312.66-second result versus about 0.09 seconds for
CGAL EPECK, with CGAL using 21.23x less RSS. This checkpoint neither reruns that
unchanged multi-hour legacy output path nor claims parity. Per-case parity or
superiority remains the explicit Phase 17 gate.

## Source and linked size

Production grows by 676 insertions and 153 deletions (net +523 lines). Embedded
and integration proof tests grow by 290 insertions and two deletions (net +288
lines). The canonical dependency-only harness was clean-built with rustc
1.97.0:

| Consumer/profile | Parent | Candidate | Delta |
| --- | ---: | ---: | ---: |
| General release native | 4,111,564 | 4,118,684 | +7,120 (+0.1732%) |
| General release WASM | 2,770,580 | 2,780,083 | +9,503 (+0.3430%) |
| Immediate release native | 4,144,780 | 4,151,900 | +7,120 (+0.1718%) |
| Immediate release WASM | 2,785,134 | 2,794,642 | +9,508 (+0.3414%) |
| General size native | 1,891,578 | 1,899,770 | +8,192 (+0.4331%) |
| General size WASM | 1,183,507 | 1,189,673 | +6,166 (+0.5210%) |
| Immediate size native | 1,903,606 | 1,911,902 | +8,296 (+0.4358%) |
| Immediate size WASM | 1,193,551 | 1,199,830 | +6,279 (+0.5261%) |

The final large probe itself grows 16,228 text bytes (0.3152%) and 20,872 file
bytes (0.3226%). The representation reuses existing identity and hash-map
instantiations; a rejected identity-keyed hash-map prototype added roughly
twice as much probe text. Current growth remains explicit debt until the
historical subdivision/trace/BSP engine is removed.

## Call graph and validation

The workspace utility, restricted to Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, and Hypermesh, reports:

| Graph | Nodes | Edges |
| --- | ---: | ---: |
| Production | 20,208 | 40,316 |
| Tests/bench/examples/fuzz evidence | 26,665 | 50,333 |

The production graph contains one constructed classifier route, one checked
recipe/intersection builder, and direct remapping/finalization. The temporary
fingerprint index has no final-graph owner, and no new Boolean orchestrator,
runtime retry, compatibility surface, or duplicate graph is present.

Final gates passed:

- default, no-default, and all-feature test matrices;
- formatting, all-target/all-feature Clippy with `-D warnings`, and all-feature
  rustdoc with `-D warnings`;
- all-feature benchmark compilation including dispatch tracing;
- all fuzz binaries and the bounded ASan/libFuzzer campaign;
- minimal/all-feature `wasm32-unknown-unknown` checks;
- eight native UI tests and locked release Trunk build;
- both-policy large topology and Massif runs;
- clean native/WASM size harnesses, fixed-binary Callgrind, pinned native
  controls, and final five-crate call graphs; and
- `git diff --check`.

## Remaining Phase 12–13 work

The current graph deliberately retains affine `Point3` materializations because
the historical consumers require them. Fully lazy construction is not complete.
Normalized source faces carry recipes, but polygons produced by the old
recursive subdivision path may have cleared retained identities and therefore
produce `None`. Positive-area coplanar overlap remains a graph marker rather
than a complete shared overlay boundary.

Most importantly, the recipes are not yet consumed by a production face
corefinement/radial engine. Phase 14 must build the shared coplanar overlay and
constrained face arrangements; Phase 15 must build radial rings, cells, and
winding classification; Phase 16 must atomically cut over and delete
subdivision, segment tracing, local BSP, and EMBER configuration/API ownership.
No phase completion, deletion, or CGAL-parity claim is made here.

## Reproduction

```sh
cargo test --locked
cargo test --locked --no-default-features
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo bench --locked --no-run --all-features
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo check --locked --target wasm32-unknown-unknown --no-default-features
cargo check --locked --target wasm32-unknown-unknown --all-features
ASAN_OPTIONS=detect_leaks=0 cargo +nightly fuzz run polygon_predicates \
  --fuzz-dir fuzz -- -max_total_time=30
valgrind --tool=massif --time-unit=B --detailed-freq=1 \
  --massif-out-file=/tmp/hypermesh-construction.massif \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
valgrind --tool=callgrind --cache-sim=no --branch-sim=yes \
  --callgrind-out-file=/tmp/hypermesh-construction.callgrind \
  target/release/examples/large_mesh_heap_probe boxes-3072-general strict
HYPERMESH_SIZE_TARGET_DIR=/tmp/hypermesh-construction-size \
  benchmarks/size-harness/measure.sh default
tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/hypermesh-ember-phase12-canonical-recipes-production \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format json,dot \
  --per-library
```
