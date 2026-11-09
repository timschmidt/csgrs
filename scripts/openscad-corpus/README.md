# OpenSCAD corpus comparison

This Linux tool collects every case in the pinned OpenSCAD test corpus using
OpenSCAD's Manifold backend and SynapsCAD's native evaluator. An agreement means
equality with the explicitly adopted emitted OBJ or SVG boundary model. It does
not certify equivalence to the original SCAD source.

The checkout layout is `csgrs`, `synaps-cad`, and `openscad-rs` alongside one
another. Initialize the parser repository's pinned OpenSCAD submodule. The
manifest contains all 524 SCAD files at `8e8df3ddae295e8667cbc9f6c4fbb7c88b0ec45b`;
the collector rejects changed files, incomplete inventories, and dirty pins.
Supply an OpenSCAD executable with the Manifold backend, Python 3.11 or newer,
Linux `prlimit`, and GNU `time`. Build the separate worker workspace:

```sh
cargo build --locked --release --manifest-path scripts/openscad-corpus/Cargo.toml
python3 scripts/openscad-corpus/run.py \
  --corpus-root ../openscad-rs/vendor/openscad/tests/data/scad \
  --openscad /absolute/path/to/openscad \
  --worker scripts/openscad-corpus/target/release/csgrs-openscad-corpus-worker \
  --output /absolute/path/to/new-results
```

Use the actual executable path if `CARGO_TARGET_DIR` changes its location.
Record the distribution checksum, worker build command and source revisions,
linked runtime dependencies, and external assets such as fonts. An optional
`--build-receipt PATH` retains the supplied build record; executable hashes
alone do not establish build attribution.

Defaults are two workers, 300 seconds per engine per case, and 4 GiB of virtual
address space per process. `--workers`, `--case-seconds`, and
`--address-space-bytes` configure external containment. Descendants inherit the
address-space limit independently; it is not an aggregate RSS or heap bound.
Process failures and external timeouts remain separate from typed kernel
exhaustion. Cancellation terminates owned process groups and retains partial
observations. A completed collection can contain differences and failures.

The output retains source and executable hashes, commands, diagnostics, raw
receipts, comparison certainty, continuation identities, and per-case status.
No corpus skiplist or display mesh is used. See the
[admission record](../../docs/research/openscad-corpus-2026-11-09.md) for the
comparison contract, completed baseline, and outstanding gaps.

```sh
python3 -m unittest discover -s scripts/openscad-corpus -p 'test_*.py' -v
cargo test --locked --release --manifest-path scripts/openscad-corpus/Cargo.toml
cargo clippy --locked --release --all-targets \
  --manifest-path scripts/openscad-corpus/Cargo.toml -- -D warnings
python3 scripts/openscad-corpus/verify_geometry.py \
  --worker scripts/openscad-corpus/target/release/csgrs-openscad-corpus-worker \
  --output /absolute/path/to/new-controls
```
