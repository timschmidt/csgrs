# E1 representation and directed-testing extension

Baseline: `74e70aa85`. Owner: Hyperspatial development tooling, reusing
Hyperreal's shared representation recipes and dispatch tracing. Properties:
P1–P4, P18–P20, P26 and P35–P39. Missing acceptance: only eight public scalar
kinds have entered the height workflow, cache-feature combinations are not yet
run here, and the recorded fuzz smoke has no sanitizer instrumentation or
coverage-guided mutation. No production geometric contract changes are intended.

Extend the workspace corpus to all 22 finite optimized certificate recipes,
verify the actual serialized certificate tags, exercise fresh and display-warmed
values, and test ordered cross-representation pairs as positive ellipsoid axes
and normal heights. Independent exact construction and certified enclosures
must validate inverse and local composition. Preserve the original fixtures.
Hyperreal owns the shared recipe source; these are workspace development tests,
not evidence of release packaging. Serde/JSON helpers are development-only.

Run the same strict supported corpus across primitive scalar-cache feature
configurations. Record the build flags and actual feature-dependent storage.
Add dispatch-path observations and representation-family selection buckets to
the adversarial campaign. Selection must retain worst-performing valid cases
and all failure outcomes rather than reward an early error as a faster success.
ASan time/RSS limits are external containment, not a proof of native heap bounds.

The first ASan attempt on the baseline compiled successfully but LeakSanitizer
failed during process inspection under sandbox ptrace restrictions before
testing an input. Its empty crash artifact is a tooling failure, not a native
geometry failure. Retry outside that sandbox and retain the actual outcome.

## Implemented corpus and selection

`tests/dispatch.rs` reuses Hyperreal's shared 22-certificate recipe inventory,
checks each actual serialized certificate tag and public structural kind, and
uses original, JSON-reconstructed and display-warmed values in the equatorial
height → ECEF → closest inverse → NED → ECEF workflow. JSON is test-only;
this is not Hyperspatial serialization support. The initial test compilation
failed for the missing development serde/JSON support before those dependencies
were added. Every production dependency remains a native Hyper crate.

All 484 ordered recipe pairs also define positive axes `a = 100 + first`,
`b = 100 + second`, and positive height `h = first + second`. An independent
normal-foot construction at latitude 45° supplies Cartesian input. The inverse
latitude and height must lie within certified `10^-30` enclosures of the
independent values, and its local anchor must map the original point to zero.
These comparison bounds are test certificates, not production tolerances.

The four primitive-cache feature configurations run the entire integration
corpus and Clippy. Assertions inspect actual binary32 cache storage and exact
rational clone-cache behavior, matching the scalar owner's documented
configurations. JSON reconstruction starts with an empty value approximation
cache. This does not claim all process-global scalar caches are cold or disabled.

The normal-offset fuzz/probe workload adds independent fresh-forward residual
checks and vector composition. It is shared development code, not a production
transform backend. The `dispatch-trace` feature enables native Hyperreal,
Hyperlimit and Hyperlattice observations. `scripts/directed.py` performs exact
lexicase selection in 22 independent first-input representation buckets, over
workflow time, dispatch count and work checkpoints. Two generations of four
candidates per bucket completed **176/176** strict workflows; 44 selections
yielded 40 distinct replay seeds. All cases used 50 work checkpoints, so that
criterion tied in this corpus; dispatch counts ranged from 1,265 to 29,596.
The timing criterion is intentionally noisy discovery pressure. No comparative
speed claim follows from these samples.

The [selection report](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-18-e1-directed/report.json)
retains source digests, compiler, build flags, seed, all selections and scores.
[Compressed raw observations](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-18-e1-directed/observations.jsonl.gz)
retain every outcome and dispatch path across seven native layers. Hardware:
Intel Xeon E5-2680 v2 @ 2.80 GHz, x86_64 Linux. Failures/timeouts are retained and
fail the driver; they cannot be selected as fast successful computations.

## Executed checks

The corpus now contains 36 integration tests plus the README doctest. All
checks below passed; raw outputs are in
[the verification directory](https://github.com/timschmidt/csgrs-evidence/tree/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-18-e1-directed).

| Check | Result |
| --- | --- |
| `bash crates/hyperspatial/scripts/checks.sh`, stable Rust 1.99 and Rust 1.95 | Formatting, default/all-feature tests and targets, doctest, strict Clippy/rustdoc, native dependency boundary, separate README source consumer, and both fuzz builds/lints pass. |
| `bash crates/hyperspatial/scripts/representation-checks.sh` | All 36 tests and all-target Clippy pass in all four cache feature configurations; 484-pair checks remain strict. |
| Updated Hyperspatial command in `scripts/adversarial.sh` | Adversarial, representations, dispatch, inverse and oracle suites pass. Unchanged workspace suites were not rerun as a full repository campaign. |
| `cargo clippy --locked -p hyperspatial --lib --all-features --target wasm32-unknown-unknown -- -D warnings` | Native WASM build/lint passes; no runtime JS binding claim. |
| `cargo bench --locked -p hyperspatial --benches` | Forward/local and inverse optimized probes complete every row with Exact certainty. No optimization or cross-library speed claim. |
| `python3 crates/hyperspatial/scripts/coverage.py` | 824/838 production lines (98.33%), 1,151/1,235 regions (93.20%), 102/102 functions. Branch coverage remains unmeasured; 100% line/region targets remain open. |
| `zizmor --pedantic --strict-collection .`, version 1.31.1 | No findings. Default offline audits; online-only checks were not run. |

CI now executes the cache matrix, directed selection and sanitized replay, and
retains logs and failure inputs with the pinned
[upload-artifact v7.0.2](https://github.com/actions/upload-artifact/releases/tag/v7.0.2)
action. Hosted artifact upload itself has not been executed locally. No push or
publication was made. The package remains unpublished, and shared workspace
fixture imports are not release-package validation.

## Sanitizer evidence and limits

Nightly Rust 1.101.0 (`8006091eb`, 2026-10-17), cargo-fuzz 0.13.2, address
sanitizer, seed `20261018`, 10-second per-input timeout, and 2048 MiB RSS limit
were used. The initial sandbox LeakSanitizer failure and successful unrestricted
1,662-execution run are retained as separate logs. The empty crash artifact
from that sandbox failure is explicitly recorded as preexisting in later
reports; it is not a new geometry failure.

A 30-second follow-up exhausted its budget while initializing the 918-input
forward corpus (975 executions, no new mutation stage). The default driver
budget was increased to 60 seconds. Corpus initialization can still consume or
extend that budget on other machines, so the raw logs must substantiate actual
mutation; elapsed time alone does not.

The final 60-second campaigns both passed without a new crash, timeout or
sanitizer diagnostic, and both logs contain coverage-increasing mutations:

| Target | Seed corpus | Executions | Final instrumented counters / features | Reported peak RSS |
| --- | --- | --- | --- | --- |
| `forward_local` | 918 original cases | 1,822 in 61 seconds | 8,198 / 24,381 | 619 MiB |
| `normal_offset` | 44 family seeds + 40 selected cases | 511 in 61 seconds | 9,200 / 23,362 | 658 MiB |

The [ASan report](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-18-e1-directed/asan-report.json)
contains exact before/after corpus bytes, source digests, commands, compiler and
artifact inventories. [Forward/local log](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-18-e1-directed/asan-forward_local.log)
and [normal-offset log](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-18-e1-directed/asan-normal_offset.log)
retain the raw observations. These counters cover the whole instrumented fuzz
binary, not the production line/branch denominator. RSS includes sanitizer
overhead and is not a native heap allocation bound. No native failure was found
to minimize or promote in this campaign.

Open: infinite-domain and 100% coverage, formal verification of these connected
entry points, native scalar work/heap bounds, independent competitive
performance comparisons with clearly matched contracts, serialization/bindings,
release packaging, and remaining Earth/astronomy stages. The named E1 corpus
establishes neither all possible scalar expressions nor the full development plan.
