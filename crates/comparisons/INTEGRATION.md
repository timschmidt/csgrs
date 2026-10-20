# Comparison harness in CSGRS

The active harness and complete tracked archive were imported from
[`timschmidt/exactcore-hyper-comparison`](https://github.com/timschmidt/exactcore-hyper-comparison)
at `1e3b1f7a8114b6bcf5078c31706719b054d87d46`.
[`integration/provenance.json`](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/comparisons/integration/provenance.json) identifies every
adapted upstream file. The accompanying tree inventory records every upstream
path, executable mode, Git object ID, and the SHA-256/size of each LFS payload.
Licenses and notices remain with their original sources.

The historical `audits/` tree and the `integration/` import inventory are
immutable evidence. Both live in the
[evidence repository](../../docs/evidence-archive.md) at their original paths,
together with a copy of the verification scripts. Run
`scripts/verify-import.py` and `scripts/consolidate-audits.py` from
`crates/comparisons` in a checkout of that repository. The script self-tests
run here. The archived corpus's scripts, absolute paths,
dependency links, and recorded results describe their original environment.
Read the corresponding record before running a historical campaign. New runs
belong in a separate output directory and must identify the current source,
oracle, compiler, flags, inputs, and limitations. An integrity check does not
revalidate a numerical or performance claim.

Frozen `.github` directories are excluded from Git-aware workflow discovery
while remaining explicitly tracked and hash-checked by the import inventory.
They are historical source data; the monorepo's active workflows remain subject
to the full security audit.

## Reproduce current checks

The [validation record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/comparisons/integration/validation/manifest.json) retains command
outputs for the current import: 78 passing native tests, 19 existing ignored
tests, and 480 successful Criterion fixture executions in `--test` mode.
These smoke executions supply no new comparative timing claim. The external
oracle's [source/header hashes](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/comparisons/integration/exactcore-inputs.json) identify the
inputs used in this run.

This harness has its own workspace and lockfile because it requires a native
C++ exactCorelib oracle. It is excluded from the default Rust workspace so a
normal geometry-crate build does not require that external toolchain. Hyper
dependencies resolve to the sibling crates in this monorepo.

Install a C++ compiler, GMP/MPFR development headers, and the separately licensed
exactCorelib source. Set `EXACTCORE_ROOT` to its `trunk` directory; the default is
`../../../exactCorelib-main/trunk` relative to this directory. From
`crates/comparisons`:

```sh
python3 scripts/verify-import.py
python3 -B scripts/test-verify-import.py
python3 -B scripts/test-consolidate-audits.py
cargo fmt -- --check
cargo clippy --locked --all-features --all-targets -- -D warnings
cargo test --locked --all-features -- --test-threads=1
cargo bench --locked --features curve,mesh,path -- --test
```

Keep the native test runner serialized: the historical oracle has a documented
concurrent deallocation failure. The `memory-profile` feature remains separate
from timed Criterion measurements. Existing ignored tests are retained known
contract differences or failures; they do not count as passing comparisons.

The archive contains five LFS payloads. Fetch those payloads when checking out
the evidence repository; `verify-import.py` rejects a pointer in place of its
full content.
Git retains executable bits but does not retain owner/group permission bits.
The legacy migration verifier's `--git-modes` option checks that portable
contract without relaxing content hashes.
After staging, `python3 scripts/verify-import.py --staged` additionally checks
every imported index blob and mode, including the canonical LFS pointers, so
line-ending or filter transformations cannot silently alter the stored archive.

## Historical manifest discrepancy

The older `audits/migration.json` covers 38,302 archived files. One of its
digests, for `workspace/EXACTCORELIB_AUDIT_PROGRESS.md`, differs from the file
in the pinned upstream commit. Both are retained unchanged. The provenance
record includes the expected and actual digests; the new import verifier checks
the pinned Git tree independently. Consequently the legacy
`consolidate-audits.py verify --archive-only --git-modes` still diagnoses this
pre-existing discrepancy. No other content or executable-bit discrepancy was
found in that legacy manifest.

The stack's [admission protocol](../../docs/admission.md) applies to new harness
work. The [properties ledger](../../docs/properties-status.json) tracks the
remaining exhaustive testing, archive revalidation, coverage, directed fuzzing,
formal proofs, and paper requirements.
