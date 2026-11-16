# Retained machine interval ownership reconsideration

**Rejected on completed-profile runtime.** The Phase 8 candidate previously
rejected only for final native/WASM growth was reproduced on CSGRS
`3bea0ac90d8e25796d93fd34953afd0a0195a85f`, Interface
`32a59de0d69436427e5b97297c4929fd908bf326`, and firmware
`2d68e82b7341db7b747b20e4ad1455a4c1113a00`. The bounded-size exception does not
relax runtime or other admission gates. No production change is retained.

Hyperreal owns the positive scalar witness derived from `NonZeroU64`;
Hyperinterval owns nonempty/positive rational intervals and their arithmetic;
Interface retains nominal values, physical units, configuration identities and
reports. Properties P1/P4/P6/P10/P25/P27/P31–P33/P36–P39. The missing owner API
was reproduced before applying the archived candidate. Its domain includes
ordered arbitrary finite rational bounds, negative/crossing/singleton sets,
full-width unsigned uncertainty and positive integer divisors. It retains the
nominal/lower/upper arithmetic order and original validation. General empty
interval arithmetic remains separate; no new resource-bound claim is made.

## Decisive evidence

All seven native profile transcripts, identities and allocation records are
byte-identical to baseline. WASM replays agree with every native case and make
zero host calls. Six cases produce complete profiles; case 4 produces the same
typed invalid-input result. No output or allocation improvement is claimed.

The first ABBA/BAAB timing run showed first-baseline warmup drift in cases 0–4
and adverse results in cases 5–6. A separately recorded warmed confirmation
runs both executables as explicit warmups before ABBA and BAAB blocks. Each
process measures 31 samples of 64 complete construction/drop operations per
case, on CPU 0 with ASLR disabled, at the same executable path and working
directory. No build or test workload runs concurrently; the broad scalar test
process tree is suspended and resumed with recorded receipts. All original,
warmup and confirmation samples remain in the evidence.

| Profile case | ABBA change | BAAB change |
| --- | ---: | ---: |
| 0 ordinary uncertainty | +1.48% | +1.54% |
| 1 zero uncertainty | +1.83% | +1.85% |
| 2 wider calibration uncertainty | +1.48% | +1.42% |
| 3 near-zero positive calibration bound | +1.29% | +1.39% |
| 4 invalid zero calibration bound | +2.35% | +2.38% |
| 5 large rational calibration | +1.22% | +1.30% |
| 6 negative travel minimum | +1.44% | +1.46% |

These are changes in the mean of the two process medians per variant/block,
not a universal statistical or causal code-layout claim. All successful cases
regress in both confirmation orders. This fails the unchanged runtime gate;
no arbitrary layout/inlining retry, policy relaxation or unrelated optimization
follows.

Configured final native file size is 810,416 → 810,864 bytes; complete `.text`
445,031 → 445,383; address-deduplicated named text 441,771 → 442,131. Final WASM
file size is 1,069,027 → 1,069,760; code-section payload grows by 729 bytes.
These preliminary metrics fit the declared local caps. Both current final
native executables are byte-identical to earlier mapped binaries, permitting
address-map reuse without a different strip policy. The initial axis frame
shrinks from 1,048 to 808 bytes, with helper-frame redistribution; this is not a
complete peak-stack qualification. Full downstream/cumulative size and flash
admission stops after runtime rejection. All 44 existing size accounts and the
fixed program baseline remain unchanged; no production allowance is consumed.

## Checks and independent repair

The candidate passes all 200 Interface core tests, including original schedule
and canonical planner assertions. Owner and Interface all-target Clippy with
`-D warnings`, both workspace formatting checks, minimal Hyperinterval tests,
isolated rational-feature checking, Rust 1.88 fixed-only thumb checking, and
owner documentation builds pass. New exact composition, GMP oracle, physical
layout/Debug/const-access and complete division-dispatch witnesses pass.

The broad Hyperreal/Hyperinterval command is **incomplete**, not passed: before
intentional termination after runtime rejection, 70 completed result lines
report 1,437 passes, including nested subprocess suites. Four additional
`jacobi_prepared` tests complete before SIGTERM. The earlier catalog failure
and build-cache collision are preserved. The latter follows accepted and
candidate checkouts sharing a target directory; clearing affected debug
artifacts resolves it without a source workaround.

The accepted revision independently fails `gmp_api_coverage` on the existing
`ratio_parts` accessor. The retained four-line catalog repair classifies its
borrowed, potentially unreduced storage view and cache-preservation contract.
Existing `borrowed_ratio_parts_do_not_canonicalize_or_populate_caches` checks
that behavior. API enumeration and both missing/stale-classification assertions
are unchanged. Candidate-only positivity-accessor classifications are removed.
This repair changes no production arithmetic or numeric benchmark coverage.

The rejected snapshots are outside active history at
`refs/interval-unaccepted/profile-owned-20261116`: CSGRS
`6b6b2ad259b83a10cc032bd33c26020edda36b81`, Interface
`c2c3b8b95f88a49bfaae992d1c2736652a403597`. Their complete patches and hashes
are retained in the evidence. All candidate production code and proposed tests
are removed from active worktrees.

After restoration, Hyperinterval reports 247 passing test completions across
44 result lines (including nested subprocess fixtures); the Hyperreal library
and catalog pass 1,053 tests, and Interface passes all 199 core tests. Strict
owner/Interface Clippy and formatting pass. The final catalog edit passes its
focused Cargo test and Clippy checks. All 41 owner executables and both native/
WASM profile artifacts reproduce baseline SHA-256 hashes; profile transcripts
and replay match exactly. All 85 prior frozen evidence stages verify unchanged.

The catalog repair is also measured in its actual optimized test executable:
1,109,056 → 1,109,120 bytes (+64, 0.00577%), with `.text` unchanged at 631,956
bytes and named text unchanged at 623,226. Growing relocation/read-only-data
sections add 24/16/16 bytes and meet their separate existing caps. Reduced
padding is not used to offset those increases. The fixed baseline has identical
Hyperreal/Cargo source for this test; its accepted-revision control fails the
missing classification and the repaired release test passes. Account 45 records
this independent test-only correction, preserving all 44 previous accounts,
caps and the fixed baseline. It leaves 16,320 bytes of cumulative file allowance;
no firmware image or flash capacity changes.

No full CI,
release, whole owner timing portfolio, successive_operations, full stack,
firmware flash-fit, push or publication claim. Shared machine-profile ownership
remains open; do not retry this unchanged candidate. The full fifteen-phase
goal remains active.

Raw evidence: `docs/evidence/2026-11-16-interval-profile-owned-reconsideration`
in the evidence repository. It retains commands, toolchains, artifact hashes,
all timing and allocation records, source patches, failures and restoration
receipts.


Review commit `270eefdc6`. Evidence `docs/evidence/2026-11-16-interval-profile-owned-reconsideration`, local archive commit
`491d20ddfd2ffb256e2e6aa3ac037ed776406f20`: 337 files / 6,789,565 bytes,
SHA-256 `5cf531276f57325dd73ecf4e92f1008b750fb8858b6b64d17d734dcbbf38c67d`.
