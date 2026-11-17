# Cancellation consumer qualification and launch controls

The scope-local candidate `ab81708aa8c44eb895043879162803a964f6ebc5`
completes the native, WASM and packaged consumer checks below. Resource
qualification remains open. These measurements extend the
[scope-local pilot](solver-cancellation-scope-pilot.md) for P5/P6/P20/P36 and
plan 14.4; they do not admit the implementation or any memory exception.
The existing bounded-growth caps and all higher-priority gates remain intact.

The missing witnesses were full consumer lifetime and cumulative accounts,
final linked images, and source/package output equivalence after the final
scope edit. No geometry source, fixture, assertion, policy or predicate order
changed in these campaigns. The fixed program baseline remains
`c9f1f9ac55216b762e9eb97672de1e66f3b8a3fb`; added-functionality accounts have
explicit local baselines rather than treating unsupported outcomes as successful
cost baselines. The wrappers' earlier charges and failures remain recorded.

## Native lifetime and cumulative measurements

The original campaign preserves 96 final images across ten source boundaries.
It executes 15,396 observations: 15,390 successes and six expected publication
failures in the earlier five-repair implementation. Its 2,760 fresh, 32-cycle
histories cover 88,320 cycles. Including the earlier wrapper and scope controls,
17,802 full output/trace comparisons and 18,726 output comparisons pass.
Six strict Clippy configurations and formatting pass. Independent analysis and
verification check raw log hashes, sources, complete outputs, traces and accounts.

The 648 resource accounts contain 7,344 metric comparisons and retain prior
positive charges. Of these accounts, 264 fail an incremental or cumulative
allowance. All observed total capacities, required headrooms and retention after
cycle two pass. All 90 native final-image metric accounts pass. These are measured
finite workloads, not arbitrary-input memory bounds or firmware qualification.

The scope boundary itself fails 39 metric checks in 22 accounts, involving
requested/usable peak or traffic. Its operation and cleanup stack changes pass;
232 operation-stack failures in the cumulative ledger belong to the earlier
five-repair-to-wrapper boundary. Scope versus five-repair operation stack ranges
from −160 to +160 bytes, with cleanup from −40 to +40 bytes. Those direct
comparisons do not replace the preserved cumulative ledger.

The largest observed requested/usable peaks are 577,406 / 613,216 bytes;
persistent footprints are 24,976 / 25,424 bytes. Operation stack reaches 23,912
bytes and cleanup stack 2,344 bytes. Traffic maxima of 572,754,228 requested and
613,045,200 usable bytes are over complete 32-cycle histories, not single calls.
Caller/thread-runtime exclusions and stack resolution remain as documented in
the original consumer protocol. Requested bytes, usable allocation bytes,
stack and process RSS are separate quantities.

## Launch arguments affect one reproduced allocation witness

A controlled factorial experiment uses the original wrapper and scope binaries
on the 64-variable, STRICT, passive-trace, rebuilt-input case after eight warmups.
For each of three launch names, both binaries run three balanced repetitions.
All 18 full outputs and traces match the original controls. Requested traffic is
8,987,904 bytes in every run; all requested epoch tuples are identical.

| `argv[0]` | Usable traffic for either binary |
| --- | ---: |
| Original wrapper path | 9,606,440 bytes |
| Original scope path | 9,629,608 bytes |
| Shared literal | 9,607,960 bytes |

Changing only the launch name reproduces the entire 23,168-byte discrepancy
on either binary. With the same name, every requested and usable epoch tuple
matches between the binaries. This establishes a launch-input confound for this
witness. It does not identify the allocator mechanism, explain other failures,
or justify discarding historical charges.

The matching-launch campaign therefore reuses all 96 immutable native images
and all 15,396 original observations with one constant launch name per consumer
kind. It preserves arguments, working directory, recorded environment overrides,
fixtures and caps. Every successful full output/trace matches its corresponding
original log; all six expected failures remain. Three fresh heap repetitions,
both stack fill patterns and every 32-cycle history remain in the matrix.

Independent verification checks all 7,344 metrics. With matched launch names,
267 of 648 accounts fail; a separate conservative ledger that retains every
legacy positive charge and failed obligation has 287 failing accounts. No prior
charge or failure is erased. All total capacities, headrooms and lifetime
retention checks still pass. The 48 default-feature matched accounts pass,
while their four historical failures remain in the conservative ledger.

At the wrapper-to-scope boundary, 48 metric checks in 26 accounts still fail;
all 26 use active trace recording. The earlier [trace attribution](solver-five-trace-attribution.md) demonstrates
allocation variation in snapshot construction.
That is a diagnostic lead, not attribution of every remaining failure or
permission to remove required trace output. The original and matching-launch
measurements are kept separately; no baseline migration is asserted.

## Source and package distribution

| Qualification | Source consumers | Packaged consumers |
| --- | ---: | ---: |
| Final native/WASM images | 64 | 12 |
| Successful controls | 3,600 | 864 |
| Expected old publication failures/traps | 336 | 0 |
| Full output/trace comparisons | 3,168 | 864 |
| Strict Clippy configurations | 12 | 12 |
| Image metric accounts / failed | 42 / 0 | 42 / 0 |

Source validation additionally checks 3,336 output comparisons, 432 complete
output domains and 264 retry domains. Package validation verifies six actual
crate archives and 452 source files against the candidate Git revision, twelve
resolved dependency configurations, the package-cache negative control, and
formatting. Every packaged control matches its source consumer exactly.

Each distribution ledger contains fourteen fixed-program metric accounts and
28 local added-contract accounts. Positive source and packaging charges remain
cumulative; all final-image capacities and reserves pass. Maximum observed WASM
linear capacity is 3,670,016 bytes. This is not a heap, retained-memory, stack or
RSS measurement. These counts describe affected validation, not the entire
workspace's release checks.

## Runner failures and declared execution limits

An audit found that the original native runner's command guard omitted Git and
image-inspection helpers. There were 15,441 recorded commands, one preserved
failed build, 23 Git calls and 576 inspection calls: 16,041 direct children,
exceeding its 16,000 cap by 41. Reused setup commands are counted once. Compiler
and linker descendants are outside this derived count. The campaign cannot
claim compliance with its entire operational envelope. This runner failure is
neither typed kernel exhaustion nor evidence of incorrect geometry; all original
observations, failures and positive charges remain preserved.

The source and package guards also omit helper calls. Audited totals are 2,288
and 971 direct children respectively, both within their separate 6,000 limits.
This retrospective count does not make those guards complete enforcement.

The matching-launch runner has one counted process-spawn site and no build,
Git or external inspection helpers. All 15,396 direct children fit its 16,000
cap. Eight isolated workers have no shared geometry, allocator or trace state;
there is no timing claim. The declared aggregate geometry payload capacity is
256 MiB, not a bound on aggregate RSS or runtime overhead. Inherited process
maximum-RSS fields are not used as incremental child memory measurements.
The original 5,400-second deadline, 60-second child limits, 64 MiB file-size
limit, storage reservations and 20 GiB development reserve remain unchanged.

Two preflight failures are retained: the original native default-feature mistake
and the matching runner's duplicate `log` keyword. Neither failed preflight ran
geometry. Reviewed corrections preserve fixtures, limits and original deadlines.
The matching analysis launcher initially required a live producer after that
producer had already completed. Its corrected preflight accepts a terminal
producer only with successful completion and an empty child registry. No
geometric execution is repeated by that correction. Offline analysis and
verification retain separate 600- and 300-second bounds.

## Remaining admission work

No runtime improvement is claimed. Quiet runtime, remaining memory failures,
broader downstream/release checks and complete admission remain open. Earlier
unscoped accounts are unchanged. Accepted totals remain 57 size accounts and
zero memory exceptions, with the inventory at 1,644 rows. The workflow parent
remains held and its original watcher terminal. No push was performed; the
full fifteen-phase goal remains active.

## Frozen evidence

Record `docs/evidence/2026-11-17-interval-cancellation-qualification`,
archive commit `e8bce8dcf528a98e2fad45f022fca70f93f84dfc`: 11 files /
199,940,791 bytes, SHA-256
`6a97f9ba7c481c85fccf1accbac053a36998e0a5edef538395c96cfcbb690b5e`.
The lossless container preserves 33,903 members (1,476,645,847 bytes), including
all 172 final images. Member hashes, lengths, modes and original paths are
verified against both the frozen inputs and decoded archive. Committed container
parts are independently checked against their hashes. Prior archive paths remain
unchanged. The three-copy storage margin plus 20 MiB fits above the unchanged
20 GiB reserve.

The archive also retains verification and recovery instructions for 608 redundant
checkout files (857,548,059 bytes) removed from an earlier record outside the
evidence repository's sparse selection. Git blobs and index tree were unchanged.
No active or queued input was removed.
