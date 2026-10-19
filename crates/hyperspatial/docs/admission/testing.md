# Hyperspatial testing integration

Baseline: `4a473fe8a`. Owner: Hyperspatial development tooling and the workspace
validation entry points; properties P1, P18–P20 and P35–P39. No production API
change is intended in this increment.

The missing acceptance boundary is reproducible execution of the native E1
corpus outside an ad hoc terminal session: default/all-feature tests, doctests,
strict lint/docs, a separate source consumer, compiler/target checks, and a
bounded-input fuzz target. On this baseline, checking
`crates/hyperspatial/fuzz/Cargo.toml` fails because the manifest does not exist.
The workspace build discovers Hyperspatial, but its tests have no dedicated CI
job. The new gates must preserve all certified-success and qualified outcomes.

The fuzz domain uses positive rational sphere/oblate/prolate axes, exact special
angles, signed heights, and small composition counts. It checks independent
surface identities and local basis orientation, then consumes the result again.
This bounded generator is not a scalar heap/time guarantee. Fuzz runs require
external time/RSS containment; building a target is not a sanitizer campaign.

The first MSRV run reproduces Clippy 1.95's `nonminimal_bool` failure at
`hyperlattice/src/matrix/core.rs:8029`. Hyperlattice owns this batch-shape
classifier (P1/P18); the change factors the same Boolean condition and corrects
its comment from three kinds to any two. It must preserve every point/direction/
unknown classification and is checked by the existing batch tests and the
downstream Hyperspatial suite. No policy or numeric operation changes.

## Validation

All final checks below passed on x86_64-unknown-linux-gnu. No push was made.
The Hyperspatial production API is unchanged; the additional assertions extend
source/receipt inspection, epoch identity, typed diagnostics and vector-anchor
mismatch coverage. The complete corpus now has 21 integration tests and one
README doctest, including 64 generated invariant cases.

| Command / environment | Result |
| --- | --- |
| `bash crates/hyperspatial/scripts/checks.sh`, Rust 1.99 stable | Default/all-feature targets, doctest, Clippy `-D warnings`, rustdoc `-D warnings`, workspace/fuzz formatting, native-dependency boundary, separate README consumer, fuzz build and fuzz Clippy pass. |
| Same driver with `RUSTUP_TOOLCHAIN=1.95.0` | All checks pass on the declared MSRV after the equivalent Hyperlattice Boolean factoring. |
| `cargo test --locked --offline -p hyperlattice --test matrix batch` | Both existing batch tests pass. The existing GMP oracle needed `m4`; a local Debian package extraction under `/tmp` supplied it after sudo installation was unavailable. |
| `cargo check --locked --offline -p hyperspatial --lib --all-features --target wasm32-unknown-unknown` | Native library compiles. No JS/runtime-binding claim. |
| `python3 crates/hyperspatial/scripts/coverage.py` | 645/655 production lines (98.47%), 803/842 regions (95.37%), 96/96 functions. The 95% line floor passes. |
| `bash scripts/bench-reports.sh hyperspatial`; Rust 1.95 `cargo bench --locked --offline -p hyperspatial --bench forward_local` | Both optimized probes complete all rows; no comparative speed claim. |
| `zizmor --pedantic --strict-collection .`, cached Zizmor 1.31.1 | No findings. Zizmor's default offline audit mode does not perform online audits. |

[Coverage data](../../verification/2026-10-18-e1-coverage.json) records every
production file, counts, source digests, compiler, features and exclusions.
The first measurement was 600/655 lines; the public provenance and diagnostic
assertions raised it without excluding source or lowering the 95% floor.
Stable LLVM branch coverage is unmeasured. The 100% line/region target remains
open, including unreachable receipt alternatives and failure propagation lines.

The compiled stable libFuzzer executable completed a deterministic seeded
smoke replay under `-runs=1000 -seed=20261018 -max_len=6 -timeout=10
-rss_limit_mb=2048`. Its 918 initial six-byte inputs were
`[a,b,latitude,longitude,axes,3]`, with `(a,b)` from `[(4,2),(2,4),(3,3)]`,
latitude indices 0..9, longitude indices 0..17, and axes 0..2 (exclusive ends).
The [raw log](../../verification/2026-10-18-e1-fuzz-smoke.txt) retains the
sanitizer/coverage warnings: this was an **uninstrumented corpus replay**, not
an ASan campaign or coverage-guided fuzzing result. An earlier unseeded smoke
run did not exercise the six-byte domain and is not credited as coverage.

The workspace adversarial driver now runs Hyperspatial's adversarial and
representation suites, and its benchmark dispatcher includes the native
forward/local probe. Dedicated CI runs stable/MSRV checks, WASM compilation
and coverage using the same local commands. The existing unchanged workspace
adversarial suites were not rerun as a full repository campaign here.

Open: instrumented ASan/lexicase campaigns and dispatch-class coverage,
comparative development-only oracles, stronger scalar memory/work containment,
WASM runtime/bindings, package publication and all remaining development stages.
This record establishes the named gates for the implemented E1 surface only.
