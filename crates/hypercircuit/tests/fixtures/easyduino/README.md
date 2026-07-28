# Easyduino fixtures

These fixtures pin the six board projects present in the Easyduino repository
at commit `53b14b66d64f25c55f88971c1c1b51656126bd30` (2026-06-28).
The three boards described by the upstream README as 2026 development work are
not repository board projects at that commit and are therefore not fabricated
into placeholder fixtures.

`upstream/` contains the unmodified KiCad PCB and project files. The source is
licensed under CERN-OHL-P-2.0; the upstream license is retained as
`UPSTREAM_LICENSE.txt`. `native/` contains versioned HyperCircuit semantic
documents with the entire supported editable import: nets, generic footprint
models, pads, placements, exact stackup dimensions, routes, vias, zones, and
native net-class policy. Every unsupported or assumed import fact remains in the
runtime `KiCadImportReport`; the integration tests require exact semantic parity
between a fresh import and each native document.

Regenerate native fixtures and run the practical materialization/HyperDRC smoke
path with:

```text
cargo run --example easyduino_fixture_generator --features "drc interchange"
```

The routine tests omit authored zones only while materializing. KiCad's cached
filled polygons are derived output, while the editable zone boundary/policy is
retained and parity-tested separately. The full slow path—including authored
zone re-pouring, boolean-unioned CAM layer images, fabrication, CAM audit, and
assembly audit—is intentionally retained as an explicit benchmark. It reports
typed exact-geometry termination per board and continues, so newly solvable
boards become visible without discarding evidence for boards that still reach
an exact blocker:

```text
cargo bench --bench easyduino_full_pipeline --features "drc interchange"
```

An unfiltered full-pipeline run compares ordinary `release_report` coverage,
finding counts, typed blockers, and independent round-trip dispositions against
`ordinary-release.json`. Regenerate it only after reviewing an intentional
release-policy or geometry change:

```text
UPDATE_HYPERCIRCUIT_EASYDUINO_RELEASE_SNAPSHOT=1 \
  cargo bench --bench easyduino_full_pipeline --features "drc interchange"
```

`manifest.toml` is the completeness ledger. It pins source and native hashes
plus semantic counts so deleting, replacing, or partially importing a board
fails deterministically. `fast-drc/` stores geometry-free, reviewable snapshots
of all 160 shared-runner dispositions and every durable finding identity for
each board. Regenerate those snapshots intentionally with:

```text
UPDATE_HYPERCIRCUIT_EASYDUINO_SNAPSHOTS=1 \
  cargo test --features "drc interchange" --test easyduino
```
