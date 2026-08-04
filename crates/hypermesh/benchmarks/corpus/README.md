# Hypermesh Boolean fixture corpus

`fixtures.toml` is the authoritative registry for correctness, competitive,
scaling, fuzz-seed, and heap fixtures used during the surface-arrangement
replacement. Every record has a stable ID, provenance, coordinate and topology
tags, policy expectations, size tier, operation set, and CGAL-common-contract
status.

The registry is monotonic. A failing or pathological input may be minimized or
replaced by a deterministic generator with the same semantic path, but it is
not removed to make a test or benchmark pass. Large external assets remain
content-addressed and opt-in; their URLs, byte lengths, hashes, and derivation
are versioned rather than embedding them in test binaries.

Current tiers are intentionally explicit:

- `micro`: fast exact feature/contact and Boolean truth cases;
- `regression`: a historical bug or behavior contract;
- `competitive`: an input in the shared CGAL EPECK contract;
- `scaling`: a deterministic family with multiple complexity points;
- `heap`: a large or XL process/allocator measurement; and
- `fuzz-seed`: a permanent seed for mutation/reduction campaigns.

`tests/corpus_manifest.rs` rejects duplicate/incomplete records, missing exact
assets, competitive cases absent from the Rust benchmark corpus, and heap rows
without a large size tier. `implementation-test-migration.toml` records the
committed historical source snapshot and maps the removed engine-specific test
families to current public behavior cases or stronger arrangement invariants.

`tests/intersection_corpus.rs` permanently exercises every public pairwise
intersection class across both predicate policies, both operand orders, and
both polygon orientations. Its exact rectangle oracle distinguishes disjoint,
point, segment, and positive-area coplanar intersections, and the
`polygon_predicates` fuzz target mutates the same dimensional contract while
checking geometry, order/orientation invariance, policy certainty, and payload
indices.

`large_mesh_heap_probe` exposes both `boxes-3072` and
`boxes-3072-general`. They contain the same 6,144 exact input triangles; the
first performs explicit convexity validation and primes the native closed-PWN
fact, while the second uses raw borrowed views so repeated-input cache effects
are visible without selecting a different engine. Every heap gate runs both
`STRICT` and `APPROXIMATE_512` and records output certainty and topology with
the process-memory result.
