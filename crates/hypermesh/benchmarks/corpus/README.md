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
without a large size tier. The implementation-test migration ledger is filled
before subdivision/trace tests are deleted.
