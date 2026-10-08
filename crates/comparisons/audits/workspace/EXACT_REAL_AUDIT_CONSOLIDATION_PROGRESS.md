# Exact-real audit code consolidation

Goal: consolidate all remaining audit code in `exactcore-hyper-comparison`.

2026-09-08: inventoried the three audit ledgers, reference metadata, eight
qualification trees, hidden workspace experiment trees, and surviving temporary
audit files. No migration process from the interrupted turn was running.

Archived 38,302 files (13,230 code/scripts; 789,754,456 bytes including support)
under `exactcore-hyper-comparison/audits`. Each file's original SHA-256, permission
mode, and old/new paths are recorded in `audits/migration.json`. Every copy and
compatibility link passed verification. Existing code in the production Hyper
crates was preserved in place. Compiler installations/build products and raw
binary profiler captures were excluded with explicit records.

Created dependency links from `audits/workspace` to the original external
checkouts; the relocated Constructible archive validator checks all 34 files
from three dependency tarballs. Cargo successfully resolves the relocated
Realistic audit package and its source targets. Standalone code scan found no
remaining regular source files at `/tmp` top level.

Validation complete: seven migration recovery/conflict/path tests pass; all
38,302 archived files and original compatibility links verify. Staged Git blobs
match original SHA-256 values (23,301 distinct blobs), including 7,193 artifacts
hidden by historical ignore files. All 274 selected Python/Node/shell drivers
pass syntax checks. Five Rust packages resolve all 13 source targets. The
relocated Realistic numerical target passes `cargo check --locked --offline`.
The final scan of selected original locations reports zero remaining eligible
regular files. Verification is reproducible with
`python3 -B exactcore-hyper-comparison/scripts/verify-audit-migration.py` and
recorded in `exactcore-hyper-comparison/audits/migration-verification.json`.

Files are staged in the comparison repo; no commit or push was made. Old file
paths remain compatibility links. External donor repositories and active Hyper
code remain in place; the archive includes the frozen experiment snapshots.
