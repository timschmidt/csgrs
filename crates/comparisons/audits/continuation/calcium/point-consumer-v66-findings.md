# Checkpoint 66 — demand-gated repair: final consumer and size qualification

The selected demand-gated repair passes the remaining isolated consumer gates.
Its completeness benefit justifies the measured costs; it is ready for live
integration and live-path regression checks, not yet a recorded retention.
The original ecosystem audit remains incomplete.

## Consumer identity and tests

Only Hypercurve was copied: 355 files / 23,841,357 original logical bytes in
`point-demand-consumer-v66/hypercurve`. Its manifest has nine path substitutions;
every other file, including Cargo.lock, is byte-identical to the retained
baseline. It shares the frozen 175-file demand solver and existing scalar
dependencies. No whole-stack copy or new dependency version.

Baseline and candidate Cargo metadata are identical after normalizing only the
two intended crate paths: 187 packages / 187 resolved nodes, with one instance
of each of the six Hyper crates. Manifest, target, feature and dependency
metadata otherwise match. The arithmetic adapter remains strict-first; an
approximate fallback still records the policy terminal explicitly.

The all-feature release test run passes at 2026-09-11T03:12:42.312Z:
1,764 passed, zero failed, nine pre-existing ignored across 46 suites.
All 1,773 test names and outcomes match the prior guarded consumer run, not
merely the aggregate counts. Formatting passes; all-target/all-feature Clippy
passes with warnings denied. The release WASM library builds with triangulation,
svg and hershey. This is a build gate, not new WASM consumer execution.

Both unchanged default-feature examples (`basic`, `arrangement`) execute
successfully using newly built demand binaries and the preserved baseline/eager
binaries; their outputs agree. These examples need not reach the repaired binary
image path, so its mathematical qualification remains the independently checked
solver/public, cold/history and native/WASM corpora from earlier checkpoints.
The six example executions and fourteen other qualification/build/metadata
captures pass; the separate evidence check passes at 03:18:50.225Z.

## Representative sizes

| Stripped example | Baseline bytes | Eager bytes | Demand bytes | Demand − baseline | Demand − eager |
| --- | ---: | ---: | ---: | ---: | ---: |
| basic | 11,012,856 | 11,013,992 | 11,014,424 | +1,568 | +432 |
| arrangement | 11,587,192 | 11,588,344 | 11,588,728 | +1,536 | +384 |

Unstripped demand deltas are +600/+448 bytes versus baseline and +24/−88
versus eager. ELF text grows +1,556/+1,548 versus baseline; data changes 0/−8;
BSS grows +2,528/+2,568. File size and memory-section totals are distinct.
Build paths, linker layout and dead-code selection contribute; these are not
isolated function sizes or a universal memory/size bound. No new cache field or
public representation is introduced.

Rechecked frozen collector sizes are separately identified, not new builds:
cold native demand is +4,712 bytes versus baseline / +4,176 versus eager; cold
WASM +793/+567; history WASM +702/+487. Their source and binary bindings match
the earlier qualified corpora. Existing numerical/cost evidence is not silently
relabelled as a new execution or benchmark.

Four new dedicated files (two ordinary binaries and their stripped copies)
occupy 50,279,984 bytes in `/tmp/calcium-point-consumer.qyV59b`. Earlier artifacts
remain untouched. The existing nonincremental, two-job Cargo cache is reused;
its growth is additional. Recorded /tmp availability after qualification is
15,424,610,304 bytes. No cleanup or deletion.

## Retention decision

Select the demand-gated version over the eager repair. Earlier independent
public qualification recovers 825 previously lost point-image witnesses in
each policy corpus and preserves the other 5,616 full records; these repeated
policies are not independent defect counts. The public refiner's contract,
polynomial-vanishing replay, containment/uniqueness obligations, strict proof
requirement and genuine Unknown controls remain intact. Approximate extrema
cannot supply an exact witness without replaying image construction under STRICT.

Compared with eager, demand avoids the ordinary-path strict endpoint probe and
removes the unchanged-result allocation/peak penalties in the native corpus.
The selected Unknown control remains about 24–25% faster in all four focused
WASM passes. Recovered-point queries do additional work, add allocations, and
some retry controls remain slower. No blanket performance or memory gain is
claimed. Eight focused tests protect rational/nonrational points, zero products,
approximate-equality/extrema rejection, half-open ownership and polynomial replay.
The reviewed production diff is +52 net algorithm lines and +374 test/helper
lines versus baseline; no API, dependency or scalar-layout change.

Under the requested priority, recovering certified answers with those safeguards
outweighs the bounded observed retry and size costs. Next apply the exact tested
solver source to the live path and run live regression/metadata gates, preserving
the existing unrelated/previously retained solver changes. Do not call the repair
retained until that live binding is verified. Power sums, remaining donor reads
and full inventory reconciliation remain open; no donor lines are added here.
