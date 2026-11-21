# Current-sample widening admission

The Phase 9 production delegation is **rejected**. One independent regression
test is retained in firmware commit `dd2b8f987ae6440dbec696836f65f1cee55c5602`;
the original widening implementation remains unchanged.

The experiment starts from firmware `2d68e82b7341db7b747b20e4ad1455a4c1113a00`
and CSGRS `13a5ed938c9ee54259699db8a2942954bb5c7fb6`. Hyperinterval owns exact
fixed Minkowski addition. FOC owns Q30 coordinate meaning, nonempty endpoints,
calibration, skew uncertainty, phase reconstruction, timing evidence and replay.
P1/P3/P6/P19/P27/P31–P33/P35–P39 apply. The reviewed gap is the private
`current::widen_interval`, called twice by `derive_phases` in channel order.

The candidate keeps negative-error rejection first and expresses nonnegative
widening as addition of `[-error,error]` through the existing Q30 adapter.
Reflection and construction of that symmetric interval cannot fail for an
already admitted nonnegative `i32` error. Lower endpoint arithmetic precedes
upper endpoint arithmetic, and overflow remains typed. No policy, cache or
scalar dispatch stage is introduced. The angle module's widened expansion and
analytic `[-1,1]` intersection have different semantics and are not changed.

The missing witness is now a retained test through the actual private helper.
It checks 140 small-grid input/error cases by enumerating every translated
member, verifies exact image bounds and consumes generated output through
reflection. Another 112 full-width cases use widened integer expectations,
including `i32::MIN`, `i32::MAX`, zero error, negative-error precedence and
endpoint overflow. Existing assertions remain byte-for-byte intact. The witness
passes both the original and candidate implementations; it does not imply that
the candidate satisfies the separate cost gates.

An external consumer exercises calibration, synchronized observation, replay
validation and Clarke conversion. All 4,108 complete result/error rows match
between baseline and candidate on Rust 1.88 and 1.95: 3,443 successes and 665
typed rejections per execution. Successful phase endpoints and zero-sequence
bounds also satisfy independent `i128` expectations. Both full 32-byte identity
digests are printed separately because their ordinary Debug format abbreviates
them. Six named rejection cases have explicit expected errors, including a bad
timing witness paired with invalid raw counts. Twelve named cases measure zero
allocation calls and bytes. Q30 interval size/alignment remains 8/4, current
sample size 160 and validated calibration size 172 bytes on this host.

Initial probe setup incorrectly tried to prepare a proof for zero skew error
while allowing nonzero channel skew. Both initial baseline executions stopped
there before timing. Their sources and failed logs remain in evidence. The
same input remains an expected `CurrentCalibration` failure through the actual
validation operation; no production guard or fixture was weakened.

The Rust 1.88 final consumer grows from 419,008 to 423,192 bytes: **+4,184**,
exceeding the unchanged **4,096-byte** incremental file cap. Rust 1.95 grows
272 file bytes; both `.text` sections grow 160 bytes. One eight-block
ABBA/BAAB timing screen was already underway when the size result was reviewed.
It records 24,379,392 completed calls, split equally between successful sample
chains and typed rejections, excluding warmups. Successful-case campaign means
are slower by about 2.74–8.77%; some early-error controls improve or remain
unchanged, and the zero-input case includes an outlier. All samples remain.
This host screen is not complete owner/platform performance qualification or
MCU-cycle evidence. The independent size failure already blocks retention.
No cap, layout or inlining workaround follows.

The exact rejected patch remains at firmware ref
`refs/interval-unaccepted/current-widening-20261116`, commit
`518a1450d138d16761c642e2a42659abd95e71a5`. Production code is restored before
the test-only candidate is qualified. Final portable tests pass **603** tests
in 55 result lines; release FOC passes **89** tests; strict all-target Clippy
and workspace formatting pass. Both native consumer binaries and the existing
Q30 integration executable are byte-identical to baseline, with identical
complete consumer transcripts.

The retained test executable grows 24,384 file bytes (0.3980%), 1,600 complete
`.text` bytes and 1,592 bytes in the address-deduplicated named text union.
These fit its predeclared 32,768-byte file and 16,384-byte code allowances, each
also limited to 4%. All other growing allocated sections satisfy the unchanged
4,096-byte **and** 1% incremental caps; `.data.rel.ro` grows 544 bytes (0.7177%).
No padding or cross-target savings offset growth. Account 46 in
[`interval-size-accounts.json`](interval-size-accounts.json) shares the fixed
program baseline, preserves all 45 earlier accounts, and leaves 8,384 file
bytes of cumulative allowance. The fixed baseline already records the same
firmware source; Hyperinterval source and manifest are unchanged from it.

This test-only allowance does not authorize firmware-image growth. No new
WASM, board-image, flash-fit, transitive stack, cycle or HIL qualification is
claimed for the rejected production candidate. Existing release gaps remain
open, as do Phase 9 and the full fifteen-phase objective. No push/publication.

Raw commands, patches, corpus, failed setup, exact transcripts, timing samples,
section/symbol measurements and final checks are in
`docs/evidence/2026-11-16-interval-current-widening` in the evidence repository.


Review commit `a64c0e58cdeaf3123d979ea243882a289c25fa2f`. Evidence archive commit
`4b083c2720876ae641eb213465fad8f7e9a634c8`: 147 files / 26,976,291 bytes,
SHA-256 `58c812f5e452dfa60e269f80e997c9685237038f6effee2bdb19d1859e5c8255`.
