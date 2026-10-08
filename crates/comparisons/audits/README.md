# Exact-real audit archive

This directory consolidates the standalone code from the exactCorelib,
constructive-real, and exact-real ecosystem audits. It contains numerical
oracles, benchmark drivers, analysis and inventory scripts, patches, frozen
before/after sources, fixtures, and the reports needed to interpret them.

The original comparison harness remains at the repository root. Hyper's
production implementations and regression tests remain in their own crates.

## Find an audit

| Location | Contents |
| --- | --- |
| `workspace/EXACTCORELIB_AUDIT_PROGRESS.md` | exactCorelib audit record |
| `workspace/CONSTRUCTIVE_REAL_AUDIT_PROGRESS.md` | Boehm/Android audit record |
| `workspace/EXACT_REAL_ECOSYSTEM_AUDIT_PROGRESS.md` | ecosystem audit record |
| `workspace/exact_reals_references.txt` | reference list |
| `workspace/exact-real-references/*-qualification/` | qualification code, snapshots, and evidence |
| `workspace/.audit-*/` | source and compatibility files from experiments |
| `tmp/` | recovered temporary probes, drivers, snapshots, and reports |
| `migration.json` | old/new paths, SHA-256, sizes, permissions, and exclusions |
| `dependencies.json` | external checkouts, pinned commits, origins, and license locations |

Search this tree with `rg --files --hidden audits` from the repository root.
Historical snapshot `.gitignore` files are preserved byte for byte, so use
`rg --files --hidden --no-ignore audits` when searching fixtures too.

## Verify and use the archive

From the repository root:

```sh
python3 scripts/consolidate-audits.py verify --archive-only
python3 scripts/consolidate-audits.py setup
node audits/workspace/exact-real-references/constructible-qualification/verify-dependencies.mjs
python3 -B scripts/test-consolidate-audits.py
python3 -B scripts/verify-audit-migration.py
```

`verify --archive-only` checks every archived file's original digest and mode.
`verify` also checks the compatibility links at the original workspace and
`/tmp` locations on the migration host. `setup` recreates relative links to the
external dependencies beside this repository, plus the links inside snapshots.
It refuses to replace an existing conflicting file. Local dependency links are
ignored by Git and can be recreated on another checkout.

On the migration host, the original audit file locations are compatibility
symlinks to this repository. Both old commands and the local dependency layout
remain available. The three workspace-root progress notes also remain readable
through these links.

Historical scripts retain their exact source bytes, including absolute paths,
CPU selections, output paths, and snapshot hash assertions. Read each campaign's
README or ledger entry before invoking it. New numerical or benchmark runs may
write to the recorded output locations; use a separate experiment checkout when
collecting new evidence. Source-hash drift and unavailable historical toolchains
retain their original meaning. Migration checks do not claim to rerun every
numerical campaign or requalify its results against current Hyper.

Compiler installations, package caches, build outputs, and raw binary profiler
captures remain at their existing locations. Their exclusions are recorded in
`migration.json`. Readable profiler reports, source patches, numeric fixtures,
and audit scripts are included. Existing donor checkouts are external dependencies;
modified donor copies and compatibility code used in experiments are preserved
with their accompanying license files. See [THIRD_PARTY.md](../THIRD_PARTY.md).

The migration is resumable: `apply` checks all selected sources and any existing
destination before copying, verifies each copy, then atomically replaces the
old file with a relative link. It never changes the saved file contents. `plan`
and `replan` are for initial inventory only; replan refuses after any file has
been archived.

The manifest includes fixture files hidden by historical `.gitignore` rules.
The consolidation stages its explicit archive file list, including those
fixtures; dependency links and build outputs are excluded from that list.
`python3 scripts/consolidate-audits.py stage` stages the same explicit selection
again after documentation or migration-tool changes. The verification script
checks staged Git blob bytes against the original SHA-256 values, parses 274
standalone audit drivers, resolves five Rust audit packages, and runs the
Constructible dependency-archive validator. It leaves historical evidence intact.

The completed migration contains 38,302 original files, including 13,230 source
and script files. Its [verification report](migration-verification.json) confirms
that all staged artifacts match their original bytes (23,301 distinct Git blobs),
all 274 selected drivers parse, all 13 targets in five Rust packages exist, and
34 dependency files match their source archives. Seven recovery/conflict tests
pass. The relocated Realistic numerical target also passes `cargo check --locked
--offline` using the existing `/tmp/realistic-audit/target` build cache.

A final rescan of every selected original root found zero remaining regular
source or support files eligible for migration. The original paths resolve
through the recorded compatibility links. The workspace callgraph tool remains
in `tools/hyper-callgraph`; it is a general support utility with no reference in
the three exact-real audit ledgers. Active donor and Hyper repositories remain
the external dependencies recorded above.
