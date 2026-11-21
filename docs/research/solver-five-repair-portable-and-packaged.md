# Complete solver consumer: native, WebAssembly and extracted packages

The isolated five-repair candidate `4a04dd9744bfecb578963b42fa3bffef5b7062b5`
completes this consumer's source and extracted-package qualification. All full
output/trace comparisons, initial image caps and headroom checks pass. This
extends the [complete-consumer resource review](solver-five-repair-resource-controls.md);
it does not admit the candidate or clear its earlier memory-growth failures.

Hypersolve owns P6/P8/P9/P19/P30 transport through proposal, alpha/domain reports,
residual-ball certification and consuming diagnostics. P36/P37 require the same
contract through the actual distribution boundary. The complete 78-assertion
fixture and original operation/input/validation body remain byte-identical.
The portable shell changes only imports, report formatting and export names
from the established shell. Every warmup and final output is validated.

## Source qualification

Thirty-eight images cover the fixed `c9f1f9ac` program baseline, current
`ee4932b4`, each repair boundary and final `4a04dd974`. Ordinary cases retain
1/4/16/64 variables, both policies, rebuilt/reused inputs, cold/eight-call-warm
state and no trace/compiled-passive/active trace. Final added cases retain rebound
input, invalid-adapter retry and STRICT → APPROXIMATE_512 → STRICT qualification.

The original four-repair source reproduces 32 expected failures: native cases
fail the original qualified-ball assertion, and corresponding WASM cases trap.
The traps are not typed solver exhaustion and do not replace the native
explanation. Other reproduction cases succeed. Incorrect results never provide
successful cost baselines.

All 2,064 successful configurations pass, with 1,800 additional complete-string
comparisons across 264 feature/argument domains, including matched native/WASM
runs. The driver finishes in about 2,726 seconds within its 7,200-second limit.
Eight feature/target combinations pass strict Clippy; formatting passes. The
isolated five-repair checkout is restored to its original branch and revision.

## Extracted-package qualification

Cargo packages and verifies Hyperreal, Hyperinterval, Hyperlattice Interval,
Hyperlattice, Hyperlimit and Hypersolve together from a fresh clean RAM checkout.
The package-cache negative control reproduces stale same-version dependency
resolution and verifies that a fresh synthetic registry consumes the changed
archive. All 452 extracted source files match their committed bytes; VCS
receipts name the intended clean revision.

The consumer keeps the same Rust files, root package name and release profile.
Its manifest uses exact version requirements and explicit patches to extracted
packages. Eight feature/target metadata graphs resolve their active Hyper
libraries only from those packages, with matching versions. The selected
consumer activates Hyperreal, Hyperlattice, Hyperlimit and Hypersolve; the two
interval packages also pass package verification. This does not claim coverage
of every optional package feature or that these versions are published.

All eight final images pass 528 controls. Each complete output and trace matches
its source-worktree control. Eight strict Clippy configurations and formatting
pass. Package builds/checks finish in about 559 seconds; verification and archive
finish about 2,121 seconds after launch, including the dependency wait.

Both WASM consumers require no imports, validate output bounds and exact UTF-8,
and clear the output slot. Maximum observed linear capacity is 3,735,552 bytes,
below the inherited 192 MiB ceiling. Linear capacity need not shrink after clear;
these observations do not establish retained geometry, allocator occupancy or
process RSS, and do not authorize a memory-growth exception.

## Final-image accounting

Builds use Rust 1.95.0, locked offline resolution, six jobs, opt-level 3, thin
LTO and one codegen unit. Native file/allocated caps remain 32 KiB AND 1%; native
text/named/executable caps remain 16 KiB AND 1%. WASM file/code caps are 32 KiB
AND 1%. Both incremental and cumulative positive charges use the fixed source
baseline denominator. Native capacity is 16 MiB and WASM capacity 8 MiB, each
with 2 MiB required reserve. These are qualification capacities, not firmware
flash layouts. Added images use absolute capacities/headroom only.

Packaging is an additional boundary in each ordinary account. Every positive
source-final → package-final change is added to the existing positive source
charges. Savings never cancel earlier charges or transfer between metrics.
All fourteen accounts pass both caps; all final-image headroom checks pass.

| Target / feature | Fixed file bytes | Source cumulative charge | Packaging charge | Final cumulative charge |
| --- | ---: | ---: | ---: | ---: |
| Native / default | 2,770,440 | 1,880 | 264 | 2,144 |
| Native / trace | 2,931,496 | 1,728 | 176 | 1,904 |
| WASM / default | 1,742,378 | 2,157 | 192 | 2,349 |
| WASM / trace | 1,896,577 | 2,157 | 176 | 2,333 |

Final packaged ordinary files are 2,772,136/2,932,952 bytes native and
1,744,727/1,898,910 bytes WASM. Cumulative named native code charges remain
1,337/1,210 bytes; WASM code charges remain 1,876 bytes for either feature.
Positive allocated-section charges remain four/twelve bytes even though the
final section totals are below their fixed baselines. The largest packaged
images are 2,990,400 bytes native and 1,937,604 bytes WASM; both include added
cases and trace. Earlier resource-observer accounts remain separate and intact.

Independent verification checks raw full strings, all original fixture and
operation bytes, image hashes, ELF sections/symbols and WASM section structure.
Package verification additionally rechecks every source/archive relation,
metadata graph and the additive packaging charges. Numerical passes do not
admit any account: accepted totals remain 57 size accounts and zero memory
exceptions. Quiet runtime, earlier memory failures, resource/cancellation,
broader distribution and complete applicable validation remain open. The full
fifteen-phase goal remains active. No push.

## Frozen evidence

Source record `docs/evidence/2026-11-17-interval-five-repair-portable`, archive
commit `674a0f5487c5be3fd85df5ffd1862f9fd3ba65ed`: 309 files / 42,478,380 bytes,
SHA-256 `bb953e6601ba5c953b9b1d00172a45e431f4cf753f22a8761525b610ad276ab7`.
Package record `docs/evidence/2026-11-17-interval-five-repair-packaged`, archive
commit `99161a0bca9019cb13425e26e4c4b635435642bf`: 140 files / 11,295,447 bytes,
SHA-256 `dff37ca2318f78975a34c2bda4e38818bc486072c866b82b5dabb09036485ced`.

Both records preserve all final images, full controls, commands, protocols,
accounts and verifiers with source/stored hashes, lengths and modes. The package
record includes six verified repository-owned archives as `.crate.gz`; no
downloaded third-party package archives are included. Earlier archive paths
and index rows remain unchanged. The original quiet watcher stays terminal,
its workflow parent held and queued commands untouched.
