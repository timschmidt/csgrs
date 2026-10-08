# Calcium / current FLINT audit — in progress

This continues, but does not close, the ecosystem audit. Source coverage,
numerical qualification and transfer decisions are separate requirements.
The full original reference inventory remains in scope.

## Latest checkpoint: remaining qqbar and generic support (84)

Thirty-seven new complete files, one partial completed, 5,324 new lines. Both
qqbar directories are fully read: 302 files / 28,491 lines. Cumulative coverage
is 1,576 complete files, 19 partial, 200,953 unique lines. Wider support and the
whole requested ecosystem inventory remain open.

Three donor defect families are reproduced: large-exponent dispatch abort,
signed root-degree overflow, and failure-path root-vector leakage. Independent
checks validate 1,078 returned algebraic values; normal native/Memcheck outputs
match with zero errors/live blocks. The failing cubic root queries retain 408
bytes per call on this build; successful controls are clean. No new donor or
Hyper production change is retained.

Hyper debug/release match 117 records and full certificates: all 160 enclosures
correct, 38 Equal and six Unknown outcomes. Fmt, Clippy, metadata and corruption
controls pass. No matched benchmark or full-stack/WASM qualification. Existing
libraries and shared Cargo storage are reused, with no source-tree copy/cleanup.
Seven continuation transfers remain retained.

```sh
node verify-qqbar-remainder-v84.mjs
```

Final sealed verification passes 2026-09-12T05:15:30.461Z: 4,498 artifacts /
101 captures (79 successful, eleven diagnostic, nine harness and two environment
failures). All commands terminal; 957 live files unchanged.
[Findings](qqbar-remainder-v84-findings.md),
[manifest](qqbar-remainder-v84-manifest.json),
[final capture](results/qqbar-remainder-verify-v84.json).

## Previous checkpoint: complex components and symbolic conversion (83)

Twenty-six new complete files / 5,728 lines cover both pinned component and
expression-conversion implementations/tests. The public importer has two confirmed
false-success families and leaked Pow temporaries. A two-clear audit-only control
removes the leak without changing results; no donor or Hyper source is changed.
Native component/round-trip checks pass, but Memcheck aborts in a donor LLL
assertion. An isolated abs call reproduces under Valgrind-none too. The cause
remains unresolved; failed memory qualification is not relabeled successful.

Hyper passes all corresponding angle and norm-enclosure checks. Debug/release
streams match all 585 records; norm equality proves 144 rows and returns Unknown
on 48. No production change, matched benchmark or full-stack/WASM qualification.
Seven continuation transfers remain retained; full ecosystem scope remains open.

[Findings](symbolic-boundary-v83-findings.md). Coverage is 1,538 complete files,
20 partial files and 195,629 unique lines. Existing libraries and shared Cargo
storage are reused. One audit-only donor translation unit is copied for the
causal control; no broad copy or cleanup.

```sh
node verify-symbolic-boundary-v83.mjs
```

Final sealed verification passes 2026-09-12T04:05:53.813Z: 4,150 artifacts /
55 captures (43 successful, ten diagnostic failures, two harness failures),
one record / 1,155 bytes and empty stderr. All commands terminal; live sources
unchanged. [Manifest](symbolic-boundary-v83-manifest.json),
[final capture](results/symbolic-boundary-verify-v83.json).

## Previous checkpoint: quadratic extraction and normalization (82)

Four new complete files / 644 lines cover both pinned quadratic extractors and
tests. All 99,587 native checks pass, including both reconstructed components and
the selected conjugate. Native/Memcheck streams match over 4,651 records; zero
errors/live blocks. Hyper proves all 934 normalized and squared identities, with
every ordering correct. Full debug/release streams and certificates match.

```sh
node verify-quadratic-extraction-v82.mjs
```

[Findings](quadratic-extraction-v82-findings.md). Nonminimal radicands are allowed
by bounded policies; the Hyper corpus still proves all identities. No new
production change, matched benchmark or full-stack qualification is claimed.
Seven continuation transfers remain retained. Two empty sandbox captures are
preserved, with written-file and driver confirmations. Existing builds are reused
without a broad copy or cleanup. Coverage is 1,512 complete files, 20 partial and
189,901 unique lines. Full ecosystem scope remains open.

Combined evidence passes 2026-09-12T03:23:43.880Z: one record / 12,042 bytes,
empty stderr, code 0 / null signal. Sealing/final verification are separate gates.
Final sealed verification passes 2026-09-12T03:26:09.135Z: 3,945 artifacts /
19 captures (17 current successes, two unusable), one record / 1,034 bytes and
empty stderr. All commands terminal; live sources unchanged.
[Manifest](quadratic-extraction-v82-manifest.json), [final capture](results/quadratic-extraction-verify-v82.json).

## Previous checkpoint: scalar boundaries and reciprocal inverses (81)

Forty-two new complete files / 1,902 lines cover both pins' reciprocal inverse,
rounding, binary-import and scalar-helper code. Native qualification passes
163,328 independent checks; Memcheck finds zero errors/live blocks and the same
19,924-record output. Hyper's 19,341 records match debug/release, with exact float
imports and all tested certified integer results correct. Reciprocal inverse
Unknowns remain explicit; derived API error distinctions are preserved.

```sh
node verify-scalar-boundary-v81.mjs
```

[Findings](scalar-boundary-v81-findings.md). Four failed harness/checker captures
and corrections remain preserved. No new production transfer or benchmark claim.
Seven retained continuation changes remain. A two-candidate rounding proposal is
unimplemented/unmeasured. Source coverage is now 1,508 complete files, 20 partial,
189,257 unique lines; remaining references and inventory reconciliation stay open.
Existing libraries and shared builds are reused; no broad copy or cleanup.
Final sealed verification passes 2026-09-12T03:02:20.667Z: 3,863 artifacts /
23 captures, one record / 1,200 bytes, empty stderr, code 0 / null signal.
[Manifest](scalar-boundary-v81-manifest.json), [final capture](results/scalar-boundary-verify-v81.json).

## Previous checkpoint: inverse-angle completeness audit (80)

Nineteen new complete files / 1,738 lines are read across both pinned donors.
Current FLINT misses the exact angle 1/3360 for `tan(pi/3360)`: its approximate
search proposes 1/3359, rejects it exactly and stops. Independent integer
polynomial division and root isolation prove the counterexample. This is a
recognition completeness failure, not an unsound accepted equality.

The main corpus passes 25,216 exact checks; Memcheck finds no errors/live blocks.
Hyper emits identical debug/release certificates in 1,849 records per profile.
Compact angle forms prove the denominator-3360 identity; some equivalent radical
forms remain Unknown. Derived acot is labeled separately. No new production
change or performance claim. Seven retentions remain; full scope stays open.

```sh
node verify-qqbar-inverse-v80.mjs
```

[Findings](qqbar-inverse-v80-findings.md). Two unusable empty captures and a
source-count reporting correction remain preserved. Existing native libraries
and the shared Rust target are reused; no broad copy or cleanup. Continuation
coverage: 1,466 complete files, 20 partial, 187,355 unique lines.

Final sealed verification passes 2026-09-12T02:06:25.019Z: 3,764 bound artifacts,
25 captures (22 current, one development, two unusable), one record / 1,228 bytes,
empty stderr, code 0 / null signal. All commands terminal; live sources unchanged.
[Manifest](qqbar-inverse-v80-manifest.json), [final capture](results/qqbar-inverse-verify-v80.json).

## Previous checkpoint: cheaper exact proof, still not retained (79)

Direct exact tan/cot forms and sparse field arithmetic preserve all prior
certificates while reducing selected repeated-query times to about 42% of the
previous candidate. They still cost roughly 4x live baseline; allocations remain
26 versus baseline 6, and failed-proof overhead persists. No production change.

Matched baseline/candidate debug/release suites pass: default 756/766, all-feature
859/870. Independent oracles and three new differential tests pass. Full metadata,
Clippy, formatting and WASM compilation pass. All 1,728 public rows/profile match
checkpoint 77. Memcheck finds no errors/lost bytes; 19,000 bytes remain reachable.

Three-way native qualification checks 65,664 observations and 3,456 preflight
records over the unchanged 96-case / 576-group corpus. Baseline/prior rebuilds
match frozen products; only two new snapshots, 4,403,848 bytes total. Shared build
cache reused. Raw failures, short rows and the answer/certificate classification
correction are preserved. No new donor credit or retained transfer.

```sh
node verify-twelfth-revision-v79.mjs
```

[Findings](twelfth-revision-v79-findings.md). Seven continuation improvements
remain retained. Proof scheduling, remaining references and full inventory
reconciliation stay open; no broad completion or universal performance claim.

Combined evidence passes 2026-09-12T01:03:24.740Z. The
[manifest](twelfth-revision-v79-manifest.json) binds 3,661 artifacts / 149 captures:
140 current successes, three development successes, two failed and four unusable
empty captures. Recorded /tmp available 13,929,951,232 bytes. Donor worktrees clean.
Final verification passes 2026-09-12T01:10:04.036Z: one record / 1,293 bytes,
empty stderr, code 0 / null signal; all commands terminal.
[Final capture](results/twelfth-revision-verify-v79.json).

## Previous checkpoint: native cost rejects retaining this version (78)

The proof's new exact answers remain valid, but repeated-query costs are too
large to select this version. A warmed unequal cotangent query goes from 0.874
to 8.029 us (paired ratio 9.188), with 6→41 allocation requests and 520→4,744
requested bytes. Case-isolated replication reproduces the large penalty.

All 40,608 main/isolated observations and 2,304 preflight records are checked,
using separate CPU/allocation binaries, balanced pairs and corrected statistics.
The 96-case / 576-group corpus retains short, slow and changed-answer rows.
Cold target queries benefit, but those Unknown→Equal results are not equal-work
speedups. Twelve corrupted records/streams rejected; full dependency graphs match.

Final verification passes 2026-09-12T00:13:42.729Z: 2,899 artifacts / 277
captures (274 current successes, two development successes, one preserved
failure), one record / 1,407 bytes, empty stderr, code 0 / null signal.
[Final capture](results/twelfth-cost-verify-v78.json),
[manifest](twelfth-cost-v78-manifest.json).

```sh
node verify-twelfth-cost-v78.mjs
```

[Findings](twelfth-cost-v78-findings.md). Revise the proof overhead without
dropping its exact answers, then rerun qualification. All live and candidate
sources remain unchanged; seven retained transfers and donor coverage unchanged.
Four small frozen binaries total 8,750,456 bytes, with existing build-cache reuse
and no new Hyperreal tree copy. Recorded /tmp available: 13,970,505,728 bytes.
No cleanup, production edit, commit or push.
Remaining source reads, WASM/consumer/size gates and full inventory stay open.

## Previous checkpoint: twelfth-turn proof candidate initially qualified (77)

An isolated, bounded Q(sqrt(2),sqrt(3)) proof now resolves all 32 identities from
checkpoint 76 without replacing the compact trigonometric nodes or their caches.
Independent polynomial/matrix/interval checks and cache/domain controls pass.
The unchanged public probe has 1,728 rows/profile: 192 repeated Unknowns become
Equal, the other 1,536 rows remain correct, and debug/release outputs match.

Matched baseline/candidate tests preserve all original membership: candidate
763 default and 867 all-feature tests, in both debug and release. Clippy,
formatting and WASM compilation pass; 126-package dependency graphs match.
Memcheck reports zero errors/lost bytes, with 21,584 bytes still reachable.
No timing, matched allocation, binary-size, WASM execution or consumer claim.

```sh
node verify-twelfth-relation-v77.mjs
```

[Findings](twelfth-relation-v77-findings.md), [manifest](twelfth-relation-v77-manifest.json).
The initial qualification binds 1,759 artifacts / 41 captures: 25 accepted
final-source, nine development successes, three failed and four unusable empty
Node completion captures. Approved checks replace the empty completion evidence;
all original logs and pre-correction sources remain preserved.
Final verification passes 2026-09-11T23:34:29.411Z: one record / 1,532 bytes,
empty stderr, code 0 / null signal; all commands terminal.
[Final capture](results/twelfth-verify-v77.json).

**Not retained.** All edits remain in the 183-file candidate, copied from only
181 Hyperreal files (5.3 MB), not the whole stack. Live 957-file state and seven
retained transfers unchanged. No new donor credit; coverage remains 1,447 complete,
20 partial, 185,617 uniquely read lines. Existing build cache reused; recorded
/tmp availability 13,994,549,248 bytes. No cleanup or production edit. Cost and
consumer gates, remaining source reads and full inventory reconciliation stay open.

## Previous checkpoint: rational-angle source audit and completeness candidate (76)

Completed 32 full donor files / 1,964 new lines across both pins: roots of unity
and forward rational-angle functions plus all selected tests. Independent exact
field/cyclotomic checks pass 28,586 assertions, including full polynomials,
both enclosure components, pole flags and root recognition. Memcheck reports
zero errors and zero live blocks; ten record/stream corruptions are rejected.

The unchanged Hyper scalar's debug/release probes agree on all 1,728 observations
per profile. Thirty-two twelfth-turn radical identities remain Unknown across
shifts/budgets; all 552 periodic equivalents, 552 unequal controls and 72 repeated
poles behave correctly. This is a candidate for certificate-preserving relation
proofs, not an implemented optimization or a new retained change. No benchmark.

```sh
node verify-qqbar-trig-v76.mjs
```

Final verification passes 2026-09-11T22:38:30.116Z: 1,429 artifacts / twenty
gates (sixteen successful, four preserved failed), one record / 1,082 bytes,
empty stderr, code 0 / null signal. Failed audit-harness formatting, type conversion
and formatting-difference verification are preserved; all commands terminal.
[Findings](qqbar-trig-v76-findings.md), [final capture](results/qqbar-trig-verify-v76.json).

Coverage is now 1,447 complete files, 20 partial and 185,617 uniquely read lines
in this continuation. Seven transfers remain retained; the 957-file live map
is unchanged and checkpoint 75's verifier remains valid. One dedicated C binary
is 13,872 bytes; Rust probes reuse the shared target without copied snapshots.
Recorded /tmp availability is 14,025,854,976 bytes. No cleanup or production edit.
Further algebraic/inverse-trig/support reads, the new relation candidate, power
sums and the full remaining reference inventory still require work.

## Previous checkpoint: certified zero-factor removal retained (75)

The exact qualified candidate is now retained in live Hypersolve, the seventh
Calcium/FLINT continuation transfer. Only the main-file diff and seven-test
module are promoted; all other 955 previously recorded identities are unchanged.
Fresh live tests pass 817 default and 818 per all-feature debug/release profile,
with complete name/outcome agreement. Clippy, formatting and release WASM build
pass. All 139 Cargo packages/nodes and locks match after intended path normalization.

```sh
node verify-zero-factor-retained-v75.mjs
```

Final verification passes 2026-09-11T22:10:23.914Z: 1,331 bound artifacts /
fourteen captured gates (twelve successful, two preserved sandbox failures),
one record / 1,275 bytes, empty stderr, code 0 / null signal. The sandbox rustc
EPERM occurred before tests; the separate approved run passes. All commands terminal.
[Retention findings](zero-factor-retained-v75-findings.md),
[final capture](results/zero-factor-retained-verify-v75.json).

The 184 additional answers and targeted native/WASM savings justify the disclosed
control/new-answer costs and representative stripped growth of 5,744 bytes each.
Those measurements and the 1,764-pass/nine-ignored consumer run belong to the
frozen byte-identical candidate, not newly rerun live paths. No universal benefit.

No dedicated executable snapshot or source-tree copy; existing build cache reused.
Post-build /tmp availability is 14,060,937,216 bytes. No deletion, commit or push.
Donor coverage and the full remaining reference inventory stay open. Power sums
remain separate and require qualification against this new retained baseline.

All commands in older checkpoint sections below describe their historical maps.
In particular, checkpoint 67 and 68–74 dynamic live checks are not current-state
entry points after this promotion; use the checkpoint-75 verifier above.

## Previous checkpoint: zero-factor consumer/size qualification (74)

The exact candidate is selected for live integration, not yet retained. All
consumer gates pass: 1,764 all-feature release tests, nine prior ignored,
46 suites and every one of the 1,773 test names/outcomes unchanged. Formatting,
all-target/all-feature Clippy and a release WASM library build pass. Both full
dependency graphs match after normalizing only intended consumer/solver paths.

Both representative stripped examples and their preserved baselines execute
successfully. Stripped files grow 5,744 bytes each (about 0.05%); ordinary files
grow 9,680/9,752 bytes. Tool versions match, but linker/path/dead-code effects
remain confounders. The examples are integration/size checks, not credited as
direct divisor-deflation coverage. A selected-range caller review preserves the
strict-first policy and documents the indirect quotient route.

```sh
node verify-zero-factor-consumer-v74.mjs
```

Final verification passes 2026-09-11T21:51:02.880Z: 84 bound artifacts /
19 successful captured gates, one record / 881 bytes, empty stderr, code 0 /
null signal. All commands terminal; earlier failures remain preserved.
[Findings](zero-factor-consumer-v74-findings.md),
[caller review](zero-factor-consumer-caller-review-v74.md),
[final capture](results/zero-factor-consumer-verify-v74.json).

Only the 355-file consumer is copied (23,841,619 original bytes), with two path
changes and shared scalar dependencies/cache. Four dedicated executable files
total 50,310,904 bytes in /tmp/calcium-zero-consumer.LrNEZu; recorded post-build
/tmp availability is 14,190,510,080 bytes. No deletion, live edit, commit or push.
Six changes remain retained. Next: exact live integration and regression gates;
power sums and the full donor/reference inventory remain open.

## Previous checkpoint: matched isolated WASM zero-factor costs (73)

All 104 workers pass: 43,776 measured rows and 3,648 pilots over 912 group/mode
combinations, with 24 balanced paired blocks per persistent/fresh instance mode.
Complete reports match qualified native evidence. All 196 same-result bounded-
deflation groups per mode are faster under both corrected interval methods:
paired ratios 0.216–0.790 overall. The selected quotient drops from 13.209 to
7.407 µs persistent and 12.630 to 7.245 µs fresh.

Six bypass combinations show small slowdowns, up to paired 3.25%. New-answer
costs are separately reported, not called equal-work regressions. All rows,
including short batches, remain. Both interval methods retain their sampling
assumptions and lack multiplicity adjustment. Fifty deliberate record/stream
corruptions are rejected; four valid controls pass.

```sh
node verify-zero-factor-wasm-cost-v73.mjs
```

Final independent statistical/source replay passes 2026-09-11T21:25:57.740Z:
430 bound artifacts / 111 successful captured gates, one record / 604 bytes,
empty stderr, code 0 / null signal. All commands terminal. The --record path
only binds captured evidence; the ordinary verifier freshly recomputes it.
[Findings](zero-factor-wasm-cost-v73-findings.md),
[final capture](results/zero-factor-wasm-cost-verify-v73.json).

No new binary or source-tree copy. Raw observations occupy 49,537,622 workspace
bytes; existing WASM modules are reused. Post-campaign /tmp availability is
15,003,926,528 bytes. No deletion, live edit, retention, commit or push.
Linear-memory capacity is not allocation/peak/RSS evidence. Six transfers remain
retained; consumer/stripped-size gates, power sums and the full inventory stay open.

## Previous checkpoint: isolated WASM zero-factor correctness (72)

Actual WASM execution preserves all 38,280 native-qualified full observations:
33,384 rational, 1,824 cost controls and 3,072 nonrational/history records. The
16,692 paired rational queries preserve 184 new answers and 16,508 unchanged
reports; repeated policies/scales are not independent defects. The independent
history oracle passes 89,984 checks, and the prior rational oracle is rerun.
Persistent full-corpus instances and fresh control instances remain distinct.

All four offline release builds, Clippy, Rust formatting and dependency identity
checks pass. Seventy-six ABI negative controls trap as expected, four legal
sequences pass (with two additional expected traps), and all 21 deliberate record
corruptions are rejected. Every saved record/order is independently replayed.

```sh
node verify-zero-factor-wasm-v72.mjs
```

Final verification passes 2026-09-11T21:08:01.072Z: 118 bound artifacts,
23 successful captured gates, one record / 707 bytes, empty stderr, code 0 /
null signal. All commands terminal; prior checkpoint failures remain preserved.
[Findings](zero-factor-wasm-v72-findings.md),
[final capture](results/zero-factor-wasm-verify-v72.json).

Four new modules total 5,818,881 bytes in /tmp/calcium-zero-wasm.ATQP07.
Raw observations occupy 53,098,727 workspace bytes. Existing source trees/cache
are reused; no new solver copy, deletion, live edit, retention, commit or push.
Recorded post-check /tmp availability is 15,003,926,528 bytes. Six continuation
transfers remain retained. Qualification timings and linear-memory capacities
are not speed/allocation measurements. Matched WASM costs and representative
consumer/stripped-size gates remain open, as does the full reference inventory.

## Previous checkpoint: isolated native zero-factor costs (71)

The candidate remains isolated. Over 456 groups / 114 cases, all 21,888 CPU,
7,296 separate allocation and 1,824 pilot observations are preserved. All 196
same-result bounded-deflation groups run at 0.210–0.809× baseline time, with
both corrected intervals below one and lower requests/bytes/peak. A selected
existing quotient falls from 8.944 to 4.957 µs. Small bypass costs and greater
new-answer work are disclosed; no universal benefit or seventh retention.

Twenty-eight mixed-workload groups show small allocation increases. A separate
seven-case diagnostic has identical counts in all 224 paired comparisons from
56 case-isolated processes. Original rows remain intact; this supports state/
history dependence without identifying a particular cache mechanism. Nine
deliberate corruptions are rejected. Statistics are freshly recomputed;
their sampling assumptions and lack of multiplicity adjustment remain explicit.

```sh
node verify-zero-factor-cost-v71.mjs
```

This verifier also invokes nm and needs normal subprocess permission. An
uncaptured sandboxed recording attempt hit nm EPERM before creating a manifest;
the unchanged approved command then succeeded. All 138 captured gates pass.
Final verification passes 2026-09-11T19:54:15.448Z: 541 bound artifacts, one
record / 767 bytes, empty stderr, code 0 / null signal. All commands terminal.
[Findings](zero-factor-native-v71-findings.md),
[final capture](results/zero-factor-cost-verify-v71.json).

Four new executables total 11,304,616 bytes, no source-tree copy. /tmp recorded
availability is 15,031,971,840 bytes. Live/candidate sources are unchanged, with
six retained continuation transfers. WASM execution and representative consumer/
stripped-size gates remain before deciding retention. The full donor inventory
is still open; no cleanup, donor credit, commit or push.

## Previous checkpoint: isolated certified divisor-zero-factor trial (70)

The completeness trial now removes unused divisor zero factors only with STRICT
nonzero interval evidence, after the existing oversized square-free reduction.
It preserves signed primitive orientation for prior nonzero resultants and
leaves source evidence and downstream proof replay unchanged. The solver-only
candidate is separate from power sums; live Hyper remains unchanged.

Across 16,692 paired queries, 184 previous Undecided/degree-rejected cases become
Transformed and 16,508 full records stay unchanged. Repeated policies/scales are
not independent bugs. The independent full signed-polynomial/interval oracle
passes 128,652 assertions / 6,441 determinant constructions. Another 384 extended
history/policy cases remain byte-identical and pass 11,248 checks.

Fresh default tests pass 817; all-feature debug/release pass 818 each, with all
prior 811 names plus seven tests. Clippy, formatting, WASM compilation and three
native Memcheck runs pass. One new-result sign expectation and its formatting
were corrected; the algorithm never changed after the initial trial. Both old
test versions and three failed captures remain preserved.

```sh
node zero-factor-final-sources-v70.mjs
node verify-zero-factor-v70.mjs
```

Final capture passes 2026-09-11T19:13:45.345Z: 145 artifacts / thirty gates,
27 successful and three preserved failed; one record / 692 bytes, empty stderr,
code 0 / null signal. All commands terminal.
[Findings](zero-factor-v70-findings.md),
[final capture](results/zero-factor-verify-v70.json).
The initial and signed-only source checkers are historical, not current entry
points. Retained live-source checking remains checkpoint 67 below.

One 175-file solver copy (6,094,998 bytes before changes) shares frozen scalar
dependencies/cache. Four new executables total 11,393,464 bytes, reusing one
baseline executable; /tmp post-build availability is 15,054,901,248 bytes.
No cleanup, donor credit, live edit, retention, commit or push. Six continuation
transfers remain retained. Matched CPU/per-query allocation, WASM execution and
representative consumer/size gates remain open, as does the full donor inventory.

## Previous checkpoint: wider coefficients and a division completeness opportunity (69)

Both unchanged solver variants preserve 3,680 query records across all 23
admitted ordered degree pairs, five construction heights, four carrier families
and two policies. The independent full signed-polynomial/value oracle passes
81,610 assertions / 1,791 distinct determinants. Input numerators reach 2,577
bits and resultant coefficients 4,639 bits. These are authored rational point
roots, not exhaustive inputs or arbitrary interval/state qualification.

An independent probe supports removing an unused zero factor from the divisor
carrier where its selected root is strictly nonzero. All seventy selected
records / forty normalized carrier pairs pass 810 assertions, preserving divisor
isolation and producing a nonzero resultant with one root in each quotient image.
This is the known zero-resultant gap, not seventy new bugs or an implemented fix.
The separate completeness trial now precedes power-sum timing. Preserve strict
nonzero evidence, original replay, signed orientation and correct sharing/degree
logic for transformed internal carriers.

```sh
node verify-power-wide-v69.mjs
```

Corrected final capture passes 2026-09-11T18:17:46.511Z: 85 artifacts / eighteen
gates, sixteen successful and two preserved failed captures, one record / 872
bytes, empty stderr, code 0 / null signal. The failure was audit metadata:
undefined optional slots became null through JSON serialization. Original code
and captures remain preserved; the explicit-null correction changes no serialized
mathematical output. [Findings](power-wide-v69-findings.md),
[corrected final capture](results/power-wide-verify-canonical-v69.json).
The original results/power-wide-verify-v69 capture remains failed, not superseded
in place. All commands are terminal; current live and candidate maps are unchanged.

Two new executables total 5,616,216 bytes, with no solver/scalar copy; input and
paired output occupy 2,838,932 / 28,016,100 workspace bytes. The shared cache is
reused; recorded /tmp availability is 15,254,159,360 bytes. No new donor credit,
retention, cleanup, commit or push. Benchmarks, consumer/size decisions and all
remaining reference work remain open. Six continuation transfers stay retained.

## Previous checkpoint: power sums qualified on the retained witness baseline (68)

The deferred power-sum constructor is rebased in an isolated 175-file solver
copy, sharing frozen scalar dependencies. No live edit or seventh retention.
The baseline already contains the witness repair; all full public results
agree, so its old 825 recoveries are not new power-sum completeness gains.

The first rebase fails Clippy on five redundant references. Its initial source,
binaries, bindings and failed captures are preserved; only those references
change in the correction. Fresh corrected qualification passes:

- 814 all-feature tests per profile, zero failed/ignored; the retained 811
  test names/outcomes persist, with three added power-sum tests.
- 4,840 signed full-polynomial cases / 1,210 independent determinants.
- All 6,441 public records under each policy, 48,059 exact checks per policy;
  all 384 nonrational/state records, 11,248 independent checks per variant.
- Warnings-denied solver/harness Clippy, formatting and release WASM compilation.
- Four focused Memcheck runs with zero errors/lost blocks and unchanged outputs;
  reachable memory remains nonzero. Lower process allocation/reachable totals
  are not per-query, peak/RSS or bounded-retention qualification.

```sh
node verify-power-rebased-v68.mjs
```

Final verification passes 2026-09-11T17:03:02.358Z: 169 bound artifacts / 43
gates, including 41 successful and two preserved failed captures; one record /
1,233 bytes, empty stderr, code 0 / null signal. All processes are terminal.
[Findings](power-rebased-v68-findings.md),
[final capture](results/power-rebased-verify-v68.json).
The corrected source binding is power-rebased-fixed-binding-v68.json; the
initial binding/checker is historical. Retained live verification remains the
checkpoint-67 entry point below. The full historical chain was not rerun.

Twelve dedicated binaries total 34,012,184 bytes under
/tmp/calcium-power-rebased.xUuvlb; only the changed initial file was archived
for the correction, not another source tree. Shared-cache growth is additional;
post-verification /tmp availability is 15,265,529,856 bytes. Nothing was deleted.
No timing campaign has run. Next: broader coefficient/carrier checks, matched
native/WASM CPU and per-query allocation, consumer/size gates and documentation
before deciding retention. All remaining donor/reference work stays in scope;
coverage and the six retained continuation changes are unchanged.

## Previous checkpoint: demand-gated point-witness repair retained (67)

The sixth continuation transfer is live in Hypersolve. Only algebraic_binary.rs
changes, byte-identically to the fully qualified isolated candidate; the other
955 recorded source/support files and prior solver edits are preserved. Recovery
is demand-gated after typed InvalidInterval, strictly proves the point witness,
and replays the unchanged refiner's proof obligations. Approximate product/
quotient extrema must also be reconstructed under STRICT.

Earlier public corpora recover 825 answers per policy while preserving 5,616
other full records. Repeated policies/histories are not independent defects.
This version avoids eager repair's measured ordinary-path allocation/peak
penalties, but recovered-answer retries add work and can be slower. Stripped
consumer examples grow 1,568/1,536 bytes; no universal speed/memory/size gain.
The diff adds 52 net algorithm and 374 test/helper lines, including eight tests.

Live debug/release solver tests pass 811 each, zero failed/ignored, with names/
outcomes matching the candidate. Clippy (all targets/features, warnings denied),
formatting and all-feature release WASM compilation pass. Full candidate/live
metadata matches after expected path normalization: 139 packages/nodes, unchanged
lockfile. Earlier consumer/numerical/memory/benchmark results remain evidence
from frozen qualified trees, not fresh live-path executions.

Current-state verification, from this directory:

```sh
node verify-point-retained-v67.mjs --point-live
```

Final capture passes 2026-09-11T03:36:46.453Z: 127 bound artifacts / ten gates,
one record / 1,449 stdout bytes, empty stderr, code 0 / null signal. Findings:
[point-retained-v67-findings.md](point-retained-v67-findings.md); capture:
[results/point-retained-verify-v67.json](results/point-retained-verify-v67.json).
All commands are terminal. Earlier scripts asserting a pre-retention live map
are historical checks, not valid current-state entry points after this promotion.
Their captures/manifests remain immutable; the new verifier checks their hashes
without rerunning the full 47-chain or complete statistical campaigns.

No new dedicated /tmp artifact or source copy was needed for live integration;
the shared two-job nonincremental cache was reused. Recorded /tmp availability:
15,381,413,888 bytes after live gates. No cleanup, donor edit, commit or push.
Coverage stays 1,415 complete / 20 partial / 183,653 read lines. Power sums,
remaining donor/reference reads and full inventory reconciliation remain open.

## Previous checkpoint: selected-demand consumer/size qualification (66)

The candidate passes Hypercurve's all-feature release tests: 1,764 passed,
nine ignored, zero failed; all 1,773 names/outcomes match the prior consumer.
Only the 355-file consumer was copied, sharing the frozen solver/scalar sources.
Complete 187-package/node metadata agrees after intended path normalization.
Formatting, warnings-denied all-target/all-feature Clippy, selected-feature WASM
library compilation and six unchanged example executions pass.

Stripped demand examples grow 1,568/1,536 bytes versus baseline and 432/384
versus eager; BSS grows 2,528/2,568 versus baseline. Build-path/linker-layout
effects are disclosed. Four dedicated binaries occupy 50,279,984 bytes under
/tmp/calcium-point-consumer.qyV59b; shared-cache growth is additional. Previously
frozen collector sizes are rechecked, not newly rebuilt.

Final capture passes 2026-09-11T03:22:18.866Z: 87 bound artifacts / 21 gates,
one record / 760 stdout bytes, empty stderr, code 0 / null signal. Findings:
[point-consumer-v66-findings.md](point-consumer-v66-findings.md); capture:
[results/point-consumer-verify-v66.json](results/point-consumer-verify-v66.json).
This pre-integration checkpoint selected the candidate but did not itself
change production. Checkpoint 67 above records live adoption and its source map.

## Previous checkpoint: residual estimator review and focused WASM diagnostic (65)

All twenty additional statistical text matches were read completely (1,371
tooling lines): no additional estimator. This resolves the frozen inventory's
65 matches, not its nonmatches, subdirectories or the whole workspace.

Four sequential default/single-threaded/single-threaded/default GC passes reuse
the frozen WASM modules, without rebuilding. All 6,912 measured batches and 192
qualification queries match full frozen native records; 1,024 empty controls
characterize the timing envelopes. The Unknown control remains about 24–25%
faster than eager in all passes; small retry costs remain. Runtime-associated
post-GC tails shrink with single-threaded GC, but counter limits and fixed pass
order preclude causal attribution or a general setting recommendation.

The initial counter-zero failure and its plan/code/data are preserved. The
amended protocol keeps zero readings and makes any zero-denominator metric
unavailable. No measured CPU reading is zero, but short qualification and empty
controls frequently return zero thread CPU: it is not a fine-resolution oracle.

```sh
node verify-point-attribution-v65.mjs
```

Analysis and full recomputation/27 corruption controls pass. The manifest binds
91 artifacts and six successful captures. Findings: point-attribution-v65-findings.md;
analysis: point-attribution-analysis-v65.json. No new /tmp artifact, Rust build,
source-tree copy, production/donor edit, allocation/consumer/size gate, retention
or cleanup. The demand repair remains isolated pending final consumer/size
qualification and a retention decision. Remaining donor/reference reads and
the original full ecosystem objective stay open.

Final capture passes 2026-09-11T02:57:01.555Z, code 0 / null signal: 91 artifacts,
three records / 2,239 stdout bytes, empty stderr; all 956 live and 175 candidate
identities recheck. Capture: results/point-attribution-verify.{json,stdout,stderr}.
All commands are terminal; no full 47-chain or 60–64 statistical recomputation.

## Previous checkpoint: early scalar/proof and polynomial statistics (64)

Nine historical datasets are corrected from 15,984 CPU rows / 333 comparisons.
All point estimates persist. The 298 conditionally qualified comparisons have
thirteen newly inconclusive claims; 34 older log comparisons remain source-
limited and the first polynomial group remains rejected for formatting overlap.
The 3,156 allocation rows recheck, with 263 instrumented timing intervals still
withdrawn. Twenty-seven rejection controls and a pathological sampler test pass.

Known costs persist: expanded-log warm Unknown about 5.06× baseline; early
sign-proof opaque work 4.33×; eager root hot-operand work 4.55× despite fresh
benefits. Erf's lost serialized sign facts remain a completeness failure.
Early versions stay unselected; later cache/fact-aware retentions are unchanged.

```sh
node verify-early-statistics-v64.mjs
```

Historical fourteen-checkpoint verification passes 2026-09-11T02:12:49.192Z;
reanalysis 02:17:18.331Z; full recomputation/controls 02:20:00.240Z. The manifest
binds 83 artifacts / five successful captures. Findings:
early-statistics-v64-findings.md; analysis early-statistics-v64-analysis.json
(990,238 workspace bytes). No new /tmp files, builds, binaries, source-tree
copies, production/donor edits, retained transfers or cleanup. No fresh Rust,
backend, numerical, Memcheck, consumer, size or benchmark execution.

Most old pilots and all per-observation timestamps were not saved. Sixteen
recorded CPU paths now contain later binaries; only the two frozen polynomial
executables match. These limitations are not repaired by recomputing statistics.
All 45 known sampler-script matches now map to reviewed dataset scopes, but
twenty other text matches and broader semantic inventory remain open, alongside
point/power-sum work, donor reads and the complete original ecosystem objective.

Captured final verification passes 2026-09-11T02:32:09.608Z, code 0 / null
signal: 83 artifacts, five gates, fifteen records / 23,254 stdout bytes,
empty stderr. The first fourteen historical records are preserved exactly.
Capture: results/early-statistics-verify.{json,stdout,stderr}. All commands are
terminal; full 60/61/62/63 statistical recomputations and the 47-chain were not rerun.

## Previous checkpoint: complex products, rank and sign-filter statistics (63)

Five historical prototype campaigns are corrected from 25,728 CPU rows / 536
comparisons; 3,216 separate allocation rows also recheck. All point estimates
persist, 486 interval endpoint pairs change and 34 directional comparisons
become inconclusive, with no new/reversed direction. Thirty corruption controls
pass. Rank's sixteen additional-answer groups remain distinct from matching
Known status; sign outcomes include certainty and stage.

All five prototypes remain isolated. The unresolved rank control is still
48.006656 times baseline, with corrected [41.739881,51.477548] and requested
bytes 47,320→1,141,240/query. V2 complex-product reaching-path benefits survive,
but bypass and peak-demand costs persist. Sign-filter allocation savings and
the mask's smaller benchmark files do not justify their sampled runtime costs.

```sh
node verify-prototype-statistics-v63.mjs
```

Reanalysis passes 2026-09-11T01:59:05.450Z; full recomputation/controls at
02:02:00.860Z. The historical 23-checkpoint rank output and four archived
complex/sign checkers recheck. The exact 24-case BigInt sign oracle is newly
recomputed; no Rust/backend/Memcheck/consumer/size or benchmark rerun. The
manifest binds 95 artifacts, distinguishing six qualified gates from four
incomplete, failed or output-missing captures. Findings:
prototype-statistics-v63-findings.md; analysis prototype-statistics-v63-analysis.json
(1,638,145 workspace bytes). No new /tmp files, builds, binaries, source-tree
copies, production/donor edits, retained transfers, cleanup/deletion or commits.

The corrected top-level inventory covers 370 .mjs files / 65 text matches,
including all 45 known LCG matches and twenty other potential matches. It
recovers fourteen legacy files accidentally excluded initially; this is not a
complete workspace or semantic inventory. Nine known scripts remain outside
corrected scopes. Point/power-sum follow-up, remaining donor reads and the full
original ecosystem audit stay open. All 956 live identities recheck unchanged.

Captured final verification passes 2026-09-11T02:06:57.692Z, code 0 / null
signal: 95 artifacts, 24 records / 35,110 stdout bytes, empty stderr. The first
23 records preserve the historical rank output exactly. Capture:
results/prototype-statistics-verify.{json,stdout,stderr}. All commands are
terminal; full 60/61/62 statistical recomputations and the 47-chain were not rerun.

## Previous checkpoint: proof/reuse, facts and monic statistics (62)

Seven accepted CPU campaigns are corrected from 24,048 rows / 675 comparisons;
39 directional comparisons become inconclusive with all point estimates intact.
A separately reanalysed 1,728-row / 48-comparison run remains rejected for its
known Memcheck overlap. Additional certified answers and matching recorded
outcomes remain distinct; known/degree equality is not full-result equality.

Selected reuse/facts benefits and substantial costs survive both corrected
methods: deep unresolved first-touch is 3.772× baseline and warm reuse 0.284×
v2; monic's worst paired point estimate is still 1.0667× baseline. The three
completeness-first retentions remain unchanged, with these costs disclosed.
All 5,040 separate allocation rows recheck. Their 441 three-block instrumented
timing intervals stay withdrawn and are not CPU evidence. Twenty corruption
controls pass. Missing historical pilots/timestamps are disclosed, not invented.

```sh
node verify-proof-facts-monic-v62.mjs
```

Historical eighteen-checkpoint verification passes 2026-09-11T01:19:25.861Z;
reanalysis 01:25:53.698Z; full recomputation/controls 01:28:30.694Z. All code 0 /
null signal with empty stderr. Historical output is preserved verbatim before
the new corrected result. Recorder binds 78 artifacts / four successful captures.
Findings: proof-facts-monic-v62-findings.md; complete analysis:
proof-facts-monic-v62-analysis.json (2,254,179 workspace bytes).

No new /tmp files, source copies, production/donor changes, retained transfers,
binaries, benchmarks, cleanup/deletion, commits or pushes. These are offline
checks, not fresh Rust/numerical/memory/consumer/size executions. Twenty-one
known sampler matches, broader statistical inventory, point-candidate follow-up,
power sums, supporting donor reads and the full ecosystem inventory remain open.
Donor coverage and all 956 current live identities are unchanged.

Captured final verification passes 2026-09-11T01:38:44.027Z, code 0 / null
signal: 78 bound artifacts, four successful captures, 19 output records /
33,573 bytes, empty stderr. The first eighteen historical records are unchanged;
the last reports corrected statistics and limits. Capture:
results/proof-facts-monic-verify.{json,stdout,stderr}. All commands are terminal.
Checkpoint 60/61 full statistical recomputations and the historical 47-chain
were not rerun.

## Previous checkpoint: retained planner/derivative statistical correction (61)

All five planner/derivative timing campaigns are corrected from 10,672 measured
rows / 199 paired comparisons. Nine historical scripts (724 lines) were read
completely. The 40-block planner follow-up is affected by the old sampler too.
All raw timings and point estimates persist; 187 interval pairs change and
seven formerly directional claims become inconclusive. The large intended-path
benefits and known 2.7% native coarsening / 5.3% warm-WASM costs survive both new
methods. No uniform speedup or reversal of those two retained changes is claimed.

Independent binomial-convolution checks verify median-interval ranks/coverage
for 6/12/40 blocks, and fourteen corrupted record/summary controls fail closed.
Archived exact numerical oracles, test membership, sizes, 1,002 allocation rows
and 320 retention rows recheck unchanged. These are offline checks, not fresh
Rust tests, timings, memory executions or sizes. Conditional sampling assumptions
and absence of multiple-comparison adjustment remain explicit.

```sh
node verify-retained-statistics-v61.mjs
```

Reanalysis passes 2026-09-11T00:59:50.331Z; recomputation/control/evidence check
passes 01:01:56.864Z, exit 0 and empty stderr. Recorder binds 48 artifacts /
three successful captures. Findings: retained-statistics-v61-findings.md;
complete corrections: retained-statistics-v61-analysis.json (353,998 bytes).
No new /tmp files, binaries, source copies, production/donor edits, retained
transfers, cleanup/deletion, commits or pushes. Other retained/historical
statistics, point-candidate follow-up, power sums and full donor inventory remain
open. Donor coverage is unchanged; no whole-audit completion claim.

Captured final verification passes 2026-09-11T01:14:54.853Z, code 0 / null signal:
48 bound artifacts, three successful captures, one output record / 8,156 bytes,
empty stderr. Capture: results/retained-statistics-verify.{json,stdout,stderr}.
All 956 live identities and relevant frozen sources recheck; all commands are
terminal. Checkpoint 60's bound evidence is checked, not its full four-campaign
reanalysis or the full historical 47-chain.

## Previous checkpoint: correction of statistical qualification (60)

The audit's own bootstrap sampler was defective: its low-bit LCG indices force
fixed residue-class counts rather than ordinary IID resampling. Historical
confidence/significance claims using it are withdrawn pending correction.
This is not a Hyper arithmetic defect. Original timings, paired/marginal point
estimates, exact-value tests and allocations remain unchanged.

Four point-family campaigns (53/56/57/59) are recomputed from all 151,296 raw
rows, covering 3,920 comparisons. Rejection-sampled deterministic bootstrap
passes independent finite-enumeration and boundary tests; sampler-free order-
statistic intervals are also recorded. Both remain conditional on suitable
independent/common-distribution timing samples, and are not multiplicity-adjusted.
269 old directional claims become inconclusive; all point estimates persist.

The apparent case-17 WASM slowdown's corrected interval now includes one.
The constructed Unknown-control benefit and recovered-point retry cost remain
directional under both methods. See point-statistics-v60-findings.md and the
complete point-statistics-v60-analysis.json for corrected comparisons. The new
analysis uses 7,317,447 workspace bytes and no new temporary files or binaries.

Forty-five historical source matches are inventoried, including retained-transfer
measurements. That is not complete impact review: their remaining confidence
claims and retention implications must be reassessed before claiming closure.
The planned runtime replay was not launched. No production/donor changes,
new retained transfer, benchmark rerun, cleanup/deletion, commit or push.

```sh
node verify-point-statistics-v60.mjs
```

Full recomputation capture passes 2026-09-11T00:21:38.479Z. Final verification
passes 2026-09-11T00:46:19.463Z, code 0 / null signal: 83 bound artifacts, four
successful captures, 175 candidate files and all 956 unchanged live identities,
with another full corrected recomputation. Thirteen output records / 60,019
bytes, empty stderr; results/point-statistics-verify.{json,stdout,stderr}.
All commands are terminal. It rechecks earlier 48–59 evidence, not the full
historical 47-chain. Earlier checkpoint
descriptions below are historical; their original confidence statements are
superseded by this notice. The full reference inventory and separate power-sum
work remain open. Source/evidence integrity is not estimator validity.

## Previous checkpoint: matched extended WASM costs (59)

The unchanged demand candidate passes full WASM workload qualification: 4,608
observations / 129,856 independent checks and complete native-reference agreement.
The balanced direct three-variant campaign adds 55,296 observations / 2,304 pilots,
with every final record checked and no dropped samples. Optimized-tier WASM,
explicit histories, setup/timing boundaries and fresh instances are documented
in point-wasm-findings.md; this is not a default-tier/browser/app benchmark.

The 512 equal-result baseline groups span paired ratios 0.801026–1.335991;
all 768 eager comparisons span 0.752653–1.234868. A constructed Unknown control
improves eager 32.331 to demand 24.882 microseconds, with a 0.767287 paired ratio.
Retry costs remain, and some paired estimates conflict in direction with marginal
medians. Preserve both: attribution needs a bounded diagnostic replay of favorable
and unfavorable controls before final consumer/size gates and retention.

Three modules add 4,843,385 dedicated /tmp bytes; raw observations use 222,399,157
workspace bytes. Existing sources/cache are reused. No production/donor edit,
algorithm revision, new retained transfer, source copy, cleanup/deletion, commit,
push or external report. No new allocator/Memcheck/consumer/representative-size
result. Whole-audit scope, five retained transfers and donor coverage unchanged.

```sh
node verify-point-wasm.mjs
```

Offline cost check passes 2026-09-10T23:59:16.257Z. Recorder binds 104 artifacts /
19 successful captures. Findings: point-wasm-findings.md. Raw checks:
results/point-wasm-cost-check.{json,stdout,stderr}. The verifier rechecks 48–58
and all 956 live identities, not the full historical 47-chain. Separate power-sum
work and the complete original reference inventory remain open.

Captured integrity verification passes 2026-09-11T00:04:12.443Z, code 0:
104 artifacts / 19 successful captures, 12 output records / 54,712 bytes, empty
stderr. Capture: results/point-wasm-verify.{json,stdout,stderr}. All commands are
terminal; no full historical 47-chain rerun or new retained transfer.

## Previous checkpoint: cold-first-query and repeated-state semantics (58)

The unchanged demand candidate passes the cold-state semantic gate: 62,208 full
queries / 1,751,136 independent exact checks, zero mathematical failures. The
three variants each use 1,152 fresh native processes and 1,152 import-free WASM
instances, nine observed calls per group, and no unrecorded query warmups.
Constructors and selected initial histories are explicit setup, not a zero-cache
claim. Demand/eager agree exactly; native/WASM records are byte-identical.

Per platform, both repairs recover 3,456 query records and preserve 6,912. The
384 first-query gains repeat the existing 128 case/policy/history combinations
over three lifecycles. Serialization after deep refinement changes four groups
to Undecided/nonisolating in baseline and both repairs alike; full serialized
inputs and exact values remain unchanged. No candidate answer/witness is lost.

Six frozen binaries add 14,358,510 dedicated /tmp bytes; 201,468,576 raw output
and metadata bytes are in the workspace. Shared cache and existing snapshots
are reused, with no new source copy, production/donor edit, retained transfer,
cleanup/deletion, commit, push or external report. Initial lint/resume failures
are preserved; final native/WASM build and lint gates pass warning-free.

```sh
node verify-point-cold.mjs
```

Findings: point-cold-findings.md. Mathematical capture:
results/point-cold-check.{json,stdout,stderr}, passing 23:13:17.230Z. The verifier
also rechecks 48–57 evidence and all 956 live identities, not the full historical
47-chain. This checkpoint adds no CPU/allocator/Memcheck/consumer/representative-
size result. Next: matched extended WASM costs and final consumer/size gates
before retention. The separate power-sum candidate and full original reference
inventory remain open; five retained transfers and donor coverage are unchanged.

Captured integrity verification passes 2026-09-10T23:22:40.971Z: 132 bound
artifacts, 30 terminal captures (27 successful / three preserved failures),
11 output records / 38,662 bytes and empty stderr. Capture:
results/point-cold-verify.{json,stdout,stderr}. All commands are terminal.

## Previous checkpoint: demand-gated recovery and matched three-variant costs (57)

The isolated demand-gated revision preserves every eager-repair public/extended
result and passes all 811 solver tests per profile, independent exact-value
checks, warning-free Clippy and focused memory checks. It attempts strict
point-witness recovery only after the existing refiner's typed InvalidInterval
result; the public refiner and its proof obligations are unchanged.

The direct three-variant campaign comprises 55,296 CPU observations, 2,304 pilots
and 6,912 separate allocation observations over all 768 groups. Demand matches
all baseline request/byte/return-live/peak counts in the 512 unchanged-result
groups. Some Unknown-endpoint controls improve about 25% versus eager. However,
same-result baseline timing ratios still span 0.881863–1.168626; recovered-point
retries add allocations versus eager, at most five requests / 345 bytes per
query. It is neither a universal speedup nor a retained transfer.

Five frozen executables add 14,950,736 dedicated /tmp bytes; only the 175-file
solver slice is copied, and unchanged dependencies, comparison binaries and
shared cache are reused. Raw history/CPU/pilot/allocation files use 230,193,025
workspace bytes. Capacity capture records 16,393,678,848 /tmp bytes available.
No live production/donor edit, cleanup/deletion, commit, push or external report.

```sh
node verify-point-demand.mjs
```

Captured verification passes 2026-09-10T22:52:56.395Z: 129 bound artifacts / 24
successful captures, 175 candidate files and all 956 unchanged live identities.
Earlier failures through 56–48 remain preserved; no full historical 47-chain
rerun. Findings: point-demand-findings.md. Integrity capture:
results/point-demand-verify.{json,stdout,stderr}.

Next verify fresh-process first queries without preconditioning, then choose
whether to further reduce retry costs. Any retained version still needs
applicable extended WASM execution/costs, final consumers and representative
size checks. Separate power-sum work and the entire original inventory remain
open; five retained transfers and donor coverage are unchanged.

## Previous checkpoint: matched native endpoint/history costs (56)

The full native matrix passes its semantic and evidence checks: 768 groups,
3,072 qualification observations / 84,864 independent exact checks, 36,864 CPU
observations, 1,536 calibration records and 4,608 separate allocation observations.
All final full results match their independently qualified records, with 256
gained and 512 unchanged groups. These are repeated policies/histories/lifecycles
of the existing completeness gap, not independent new defects.

The broader corpus exposes costs absent from the earlier rational controls.
Same-result paired ratios range 0.902928–1.348403; 188 groups request more
allocations/bytes and 16 have a 176-byte higher measured peak. Same-result net
return-live demand is unchanged. Changed-result groups perform additional
certified work and are not equal-work speedups. See point-history-findings.md
for per-group interval limits, exact lifecycle boundaries and resource counts.

Do not retain the eager repair at this checkpoint. Next investigate demand-gated
witness recovery, preserving strict equality, whole-image and refinement proof
obligations; qualify the revision before relevant extended WASM costs and an
explicit retention decision. The prior candidate and all measurements remain
preserved. Separate power-sum work and the full reference inventory remain open.

Four frozen native executables total 12,772,864 bytes; existing source snapshots
and the Cargo cache are reused. Raw evidence uses 158,331,284 workspace bytes;
the capacity capture records 16,467,509,248 bytes free in /tmp. No production or
donor edit, cleanup, deletion, commit, push or external report. Five retained
continuation transfers and donor coverage remain unchanged.

```sh
node verify-point-history.mjs
```

The evidence check rechecks all 956 live sources and earlier failures through
55–48, not the full historical 47-chain. Numerical/statistical capture:
results/point-history-check.{json,stdout,stderr}. Integrity capture:
results/point-history-verify.{json,stdout,stderr}, passing at 22:21:23.086Z and
binding 91 artifacts / 15 successful captures.

## Previous checkpoint: nonrational/Unknown values and histories (55)

The guarded repair passes 21,216 independent exact checks over 384 queries per
variant, covering radical, algebraically obscured and trigonometric-Unknown
endpoints under two policies and four histories. Full serialized computational
values are interpreted independently; exact source recipes, rational carrier
shape, polynomial coefficients up to rational scale, selected roots, witnesses
and interval images are checked. There are 128 gained answers and 256 unchanged
query records, not 128 additional independent defects or completed cost results.

Deep refinement enables four case/policy combinations in both variants; other
histories remain Undecided/nonisolating. Matched costs must keep those histories
separate. Native/Memcheck outputs agree exactly; both have zero errors/lost
blocks, but candidate reachability is 48,648 versus 47,632 bytes. These are whole-
collector counts including serialization and extra answers, not marginal costs.

The omitted InvPi/Sqrt2 decoder cases, failed probe and original source are
preserved. The initial successful 17,640-check oracle is preserved separately
from the strengthened 21,216-check gate. The collector's unnecessary-mut warning
also remains explicit. Its draft timing modes have not been exercised/accepted.

Two executables add 6,379,848 bytes; existing snapshots and the Cargo cache are
reused without a new source copy. About 16 GB remains free on /tmp. No production
or donor change, cleanup, deletion, commit, push or external submission occurred.

```sh
node verify-point-extended.mjs
```

Captured verification passes at 2026-09-10T21:30:31.317Z: 73 artifacts / 15
terminal captures, all 956 unchanged live hashes, and prior evidence/failures.
This is not a full 47-chain rerun. Details: point-extended-findings.md.
Capture: results/point-extended-verify.{json,stdout,stderr}. Next: finalize and run
matched full-result CPU/allocation/history costs and relevant WASM qualification
before deciding retention. The separate power-sum transfer and entire ecosystem
audit remain incomplete; all five retained transfers and donor coverage persist.

## Previous checkpoint: guarded consumers and native/WASM witnesses (54)

The exact guarded source now passes 1,764 Hypercurve release tests (nine existing
ignored), all-target/all-feature Clippy and a selected-feature WASM consumer
build. The full 6,441-record arithmetic corpus executes on native and WASM under
STRICT and APPROXIMATE_512. Each preserves exactly 825 gains and 5,616 unchanged
records, with full outputs byte-identical across platforms and policies. The
approximate collector differs at exactly two query call sites, exercising the
guard's replay branch. These rational cases do not qualify unresolved endpoints.

Two stripped curve examples grow 1,136 / 1,152 bytes. The unchanged scalar example
shrinks 784 bytes, demonstrating build-path/layout effects; no purely algorithmic
byte change or general size improvement is claimed. Native/WASM collectors grow
1,152 / 560 bytes under STRICT and 1,080 / 547 under approximate policy. No new
CPU/allocator measurements; command execution durations are not benchmarks.

The candidate remains isolated pending nonrational/Unknown endpoint, broader
cache-history and WASM performance qualification. The earlier debug-consumer
result belongs to unstrengthened v2, not the final guard. Five retained transfers,
956 live source hashes and donor coverage remain unchanged. Fourteen new dedicated
files total 68,928,629 bytes; old baseline apps and the shared cache are reused.
About 16.6 billion bytes remained available on /tmp. No deletion or production edit.

Run here:

```sh
node verify-point-qualified.mjs
```

Captured verification passes at 2026-09-10T20:56:13.407Z, binding 179 artifacts /
45 successful captures, checking all live hashes and preserving earlier failures.
This checks 48–53, not a full 47-chain rerun. Details: point-qualified-findings.md.
Capture: results/point-qualified-verify.{json,stdout,stderr}. The complete original
ecosystem audit and separate power-sum performance experiment remain open.

## Previous checkpoint: certified point-image witnesses (53)

The isolated guarded repair recovers all 825 previously rejected point images
and leaves 5,616 other public records unchanged. All 5,126 returned roots and
1,794 exact witnesses pass the unchanged independent polynomial/Sturm oracle.
The original failed checkpoint-52 gate remains preserved. STRICT equality is
required for a witness; multiply/divide additionally replay whole-image
construction under STRICT when approximate ordering was allowed.

Final guarded solver: 811 all-feature tests pass per debug/release profile,
24 focused tests, Clippy/all-targets and changed-file formatting pass. The earlier
witness variant passes 1,764 Hypercurve debug tests with the existing nine ignored;
this is not yet guarded-consumer qualification. A redundant test cast, a failed
probe precondition and a six-byte memory-total binding mismatch are preserved as
test/bookkeeping failures, not mathematical defects. No oracle/output was changed.

Paired CPU: 3,840 observations plus 160 calibration records; separate allocation:
480 observations. Seventy same-result groups have unchanged allocation/live/peak
counts, with paired timing ratios 0.9302–1.0804. Ten groups gain certified answers
and are not equal-work speed comparisons. Focused memory has zero errors/lost
blocks, but 728 additional reachable bytes. No general speedup or zero-live claim.

The candidate is not retained. Next gates: guarded consumers, nonrational/Unknown
endpoint/state-history costs and representative native/WASM application sizes.
The separate power-sum optimization remains unselected and untimed. No new donor
coverage: 1,415 complete files / 20 partial / 183,653 unique read lines. All five
retained transfers and 956 live-source hashes remain unchanged.

Six dedicated point-image executables use 56,943,680 bytes in one bounded `/tmp`
directory, with the shared cache reused. About 17 GB remained free. No cleanup,
deletion, production/donor edit, commit, push or external report. Details:
point-image-findings.md.

Run here:

```sh
node verify-point-image.mjs
```

Captured integrity verification passes at 2026-09-10T20:16:38.251Z. It binds 154
artifacts and 33 terminal qualification/cost captures, checks all unchanged live
hashes and checkpoints 48–52, and preserves historical failures. This is not a
full 47-chain rerun; its latest full capture remains 04:56:34.285Z. Capture:
results/point-image-verify.{json,stdout,stderr}. All handles are terminal.

## Previous checkpoint: power sums and point-image completeness (52)

The isolated power-sum/Newton candidate passes 4,840 independent exact signed
full-polynomial cases, 19 focused debug tests and 805 default-feature debug
library/integration tests. Public qualification finds a shared pre-existing
completeness gap: 825 valid singleton images are rejected because the binary
image helper drops their exact witness. All 4,301 returned results pass the
independent root checks; baseline/candidate complete records match. The failed
mathematical gate remains exit 1, with 43,109 of 43,934 assertions passing.

The next action is a separate point-image witness repair off the retained
baseline. Power sums remain isolated, unselected and not performance-qualified.
No CPU/allocation campaign ran; collection elapsed times and unstripped driver
sizes are not benchmark/application-size evidence. Focused Memcheck reports zero
errors and no definite/indirect/possible loss, but 1,364,464 bytes in 11,605 blocks
remain reachable. No zero-live, peak-memory or baseline memory claim is made.

One 956-file isolated source copy reuses the frozen baseline and shared build
cache. Five dedicated executables occupy 34,189,432 bytes in a bounded `/tmp`
directory. Existing evidence and all five retained transfers are unchanged; no
cleanup/deletion, donor edit, commit, push or external report. No new donor lines:
coverage stays 1,415 complete files, 20 partial files, 183,653 unique read lines.
Details: power-sums-findings.md.

Run here:

```sh
node verify-power-sums.mjs
```

The captured integrity verifier passes at 2026-09-10T18:50:23.896Z, binding 55
artifacts, 957 candidate source files, five executables and ten terminal gates.
It rechecks all 956 unchanged live identities and checkpoints 48–51, preserving
both earlier and current mathematical failures. It does not rerun the full 47
historical chain. Capture: results/power-sums-verify.{json,stdout,stderr}.
All command/session handles are terminal. The full ecosystem remains open.

## Previous checkpoint: algebraic arithmetic and relation boundaries (51)

54 full-file reads / 7,223 new lines cover both selected arithmetic and relation
implementations and 22 associated test files. The 21 provisional reads from the
report handoff are included once, not added twice. Combined Calcium/FLINT
coverage: 1,415 complete files, 20 partial files, 183,653 uniquely read lines.
This does not complete qqbar, its recursive support, or the ecosystem inventory.

The independent BigInt oracle checks a sixteen-value biquadratic field corpus
in initial and explicitly cached states. All 40,352 assertions pass, including
7,344 full minimal polynomials and root enclosures, 1,008 full composed
annihilators, 2,560 polynomial-value relations, and input preservation. Binary
and unary aliases, signed powers, affine maps and rational polynomial evaluation
are exercised. The upstream test files were read, not newly executed; direct
LLL and archived runtime qualification remain separate.

Native and focused Memcheck output match exactly, 2,032,198 bytes each. Memory
instrumentation reports zero errors/live blocks. One 22,960-byte executable
reuses the existing native libraries; there is no Rust/dependency rebuild,
source copy, cleanup/deletion or production/donor edit. All five retained
changes remain unchanged. Details: qqbar-arithmetic-findings.md.

Live Hypersolve uses integer resultant samples, flat Bareiss elimination and
factorial-scaled interpolation. Power-sum/Newton construction is a genuine
alternative to prototype and benchmark. It must preserve reducible/repeated
carriers, zero-root divisor fallback, degree limits, policy and root-evidence
contracts. No Hyper prototype, matched benchmark or new speedup is claimed.

Run here:

```sh
node verify-qqbar-arithmetic.mjs
```

The captured verifier passes at 2026-09-10T18:08:04.634Z. It binds 28 artifacts,
54 source records, five successful execution/check gates, libraries/configuration
and the executable; it rechecks all 956 retained live files and checkpoints
48–50, including the preserved checkpoint 50 mathematical failures. This is not
a full 47-checkpoint historical-chain rerun; that latest capture remains
results/mpoly-bridge-verify-full.json at 04:56:34.285Z. Current capture:
results/qqbar-arithmetic-verify.{json,stdout,stderr}. All handles are terminal.

## Previous checkpoint: algebraic representation and exact decisions (50)

92 new complete-file reads / 8,646 unique lines cover both qqbar headers/manuals
and selected representation, refinement, comparison, construction and test files.
Combined Calcium/FLINT coverage: 1,361 complete files, 20 partial, 176,430 lines.
Algebraic arithmetic/relations, recursive support and the original ecosystem
inventory remain incomplete. Details: qqbar-decisions-findings.md.

The independent valid-input oracle checks 49 signed-square-root complex values,
all 4,802 ordered pairs across two cache states, copies, readonly input
preservation and 294 enclosures at three precisions. Of 35,826 assertions,
35,562 pass and 264 fail: 168 equal-nonreal root-order results and 96 missed
exact-component polish results. These are two contract issues, not 264 defects.
All tested enclosures contain the correct values and meet the checked accuracy;
ordinary equality/sign/component/magnitude decisions pass. No wrong root list
or distinct-value order is demonstrated. The mathematical gate remains exit1.

Focused Memcheck exits0 with zero errors/live blocks. Native/Memcheck stdout
is identical, 658,089 bytes each. One 18,152-byte executable reuses the existing
libraries; no broad build, source copy, cleanup/deletion or production change.
The initial JSON-binding bookkeeping failure and original scripts are preserved;
its correction changes no mathematical oracle or output. All five retained
continuation transfers remain unchanged; no new matched benchmark is justified.

Run here:

```sh
node verify-qqbar-decisions.mjs
```

This passes source/evidence integrity at 2026-09-10T05:49:16.590Z, explicitly
checking the failed mathematical gate, clean scoped memory evidence, 33 bound
artifacts, 92 source records and all 956 unchanged retained live files. It
imports49/48, not the full47 historical chain. Latest full-chain capture remains
results/mpoly-bridge-verify-full.json at04:56:34.285Z. Current capture:
results/qqbar-decisions-verify.{json,stdout,stderr}. No active handles.

## Previous checkpoint: expression formatting, builtin symbols and streams (49)

Both complete expression/builtin directories, their headers/manuals and calcium
support directories/headers are now read: 60 archived and 62 current files,
22,843 source lines. Checkpoint 49 adds 13,667 unique lines in 25 records:
24 new complete files and completion of one old partial header. Combined
Calcium/FLINT coverage is 1,269 complete files, 20 partial files, 167,784 lines.
Recursive support, algebraic paths and the full original ecosystem remain open.

Static checks pass all 474 paired builtin entries, indexes/order, 405 documented
names and 35 callback definitions used by 269 rows. This is metadata consistency,
not numerical capability or runtime formatting qualification. Display can perform
substitution and formal normalization; rendered assertions are not proofs. Literal
text, escaping and ownership concerns prevent treating the donor formatter as a
verified serializer. No new Hyper transfer or matched benchmark is justified.
Details: fexpr-formatting-findings.md.

Run here:

```sh
node verify-fexpr-formatting.mjs
```

The source/metadata check passes at 2026-09-10T05:26:43.602Z. It rechecks48,
all956 unchanged live files and recorded47 evidence bindings, not the entire
47-checkpoint historical chain. Capture:
results/fexpr-formatting-verify.{json,stdout,stderr}. No new donor runtime,
numerical, Memcheck, benchmark or size result; all prior evidence is preserved.
No production/donor change, new/tmp file/build, cleanup/deletion, commit or push.
Five continuation changes remain retained. No active handles; about19GB free on/tmp.

## Previous checkpoint: expression representation and numerical interfaces (48)

Both fexpr headers/manuals and both directories except write_latex.c are now
source-read. Checkpoint 48 adds 8,321 unique lines in 88 records: 87 new files
and completion of a partial manual. Combined coverage: 1,244 complete files,
21 partial files, 154,117 unique lines. Builtin headers/tables, LaTeX implementations,
recursive support and the full original reference inventory remain incomplete.

Flat syntax storage, borrowed views and structural replacement do not replace
Hyper's shared exact graph and cached refinement. Exact rational/dyadic imports
overlap existing capabilities. Capped decimal-conversion success does not certify
requested accuracy; formal normalization does not establish authored or selected-
root domains. No new production transfer is justified. Detailed analysis:
fexpr-representation-findings.md.

This checkpoint is source-only: tests read, not run; no new numerical, benchmark,
allocation, Memcheck or size claim. Existing failed and successful evidence is
preserved. No new /tmp files or builds, production/donor edits, cleanup/deletion,
commit or push. Five continuation improvements remain retained.

Run the new source/recorded-evidence integrity check here:

```sh
node verify-fexpr-representation.mjs
```

It passes at 2026-09-10T05:13:02.958Z, checking 88 donor records, 956 unchanged
live files and checkpoint 47 evidence bindings. Capture:
results/fexpr-representation-verify.{json,stdout,stderr}. This is not a new full
historical-chain run; the latest full-chain capture is still checkpoint 47 below.
No active handles. About 19GB remains free on /tmp.

## Previous checkpoint: archived rational functions and expression bridges (47)

All 44 archived rational-function implementation/test/header files are now read;
with the earlier manual, the complete 45-file archived slice is source-read.
Both expression bridges, current generic adapter/test and four expression helpers
are also read. Total new credit: 5,029 lines, 54 complete/two partial additions.
Combined Calcium/FLINT coverage: 1,156 complete, 22 partial, 145,796 unique lines.
Recursive support and the full original ecosystem remain incomplete.

The archive/current comparison finds the same staged denominator/content GCD
cancellation, with newer scalar fast paths. Hyper already has corresponding
rational staging and selected-root proof obligations. The formal expression
normalizer treats leaves as independent; it does not prove relations or preserve
authored denominator domains. No new production transfer or benchmark claim.
Detailed source/contract analysis: mpoly-bridge-findings.md.

Independent full BigInt polynomial/canonicality checks pass 7,308 complete values,
2,052 primary certificates and 2,142 generic-operation aliases. Nine contexts
cross 1/2/4 variables and three monomial orders; bounded signed powers, numerator/
denominator extraction, direct/normalized roundtrips and authored identities.
This is not a selected-root domain proof, malformed-input test, general GCD proof,
archived runtime or full generic-ring-suite qualification. Those tests were read,
not newly run. Original failures remain preserved and unrerun.

Native/Memcheck stdout is identical, 864,384 bytes each. Focused Memcheck has zero
errors/suppressed/live blocks; all 603,105 allocations freed, 11,974,118 cumulative
bytes including setup/checks, not donor-only cost or peak/RSS. One 28,488-byte
executable reuses existing libraries; paired logs total 1,728,768 workspace bytes.
No production/donor edit, broad rebuild, cleanup/deletion, commit or push.

mpoly-bridge-experiment.json binds 30 artifacts, five successful gates, 56 read
records, all 956 unchanged live hashes, libraries/configuration and executable.
Run here:

```sh
node verify-mpoly-bridge.mjs --e-plan-live
```

All 47 verifiers pass at 2026-09-10T04:56:34.285Z: exit0, empty stderr, 48 records/
139,322 bytes; the first47 records/135,764 bytes exactly match checkpoint46.
Capture: results/mpoly-bridge-verify-full.{json,stdout,stderr}. Five continuation
changes remain retained; no new change this checkpoint. About19GB remains free
on/tmp. Continue recursive support, generic/default helpers, fexpr/algebraic work,
all outstanding original references and full inventory reconciliation.

## Previous checkpoint: multivariate rational-function architecture (46)

All 36 current fmpz_mpoly_q implementation/test/header/manual files are read,
3,564 lines. Archived manual and supporting API/test-driver reads bring the
checkpoint to 3,955 new lines, 38 complete files and two partial files. Combined
Calcium/FLINT: 1,102 complete, 20 partial, 140,767 unique read lines. Archived
implementation, generic-ring/fexpr bridges and recursive polynomial/GCD support
remain separate requirements, as does every original uncompleted reference.

The substantive ideas are denominator-GCD staging, restricted residual
cancellation, scalar-content shortcuts and opposite-side cross-cancellation.
Hyperreal already uses rational cross-cancellation and possible-divisor
reduction; Hypersolve has common-denominator and bounded residue arithmetic.
Its partial Real coefficients and selected-root domain obligations differ from
a total formal rational-function field. Existing rational-image cancellation
is certificate-guarded. No new production transfer or performance claim selected.

Independent BigInt full-polynomial and known primitive-linear-factor certificates
pass for 23,841 complete values, including 13,797 public aliases. The 10,044
primary certificates cover twenty recipes in each of nine contexts: 1/2/4
variables and all three monomial orders, scalar/common factors and 66/130-bit
integer contents. Canonicality is independently proved for this structured
corpus, not all multivariate GCDs; no point-only or residue-only oracle is used.
Full input preservation, value properties, content and used-variable masks pass.

All 15 upstream tests also pass without filters at explicit multiplier one.
Their lexicographic/random/shared-backend references are not independent; the
separate corpus closes that qualification gap for its stated cases. Valid
generated string roundtrips run; malformed/failing inputs remain unqualified.
Native and focused Memcheck outputs are identical, 3,087,444 bytes each. Memcheck
reports zero errors/suppressed/live blocks; all 2,797,045 allocations freed,
75,163,303 cumulative bytes including checks/setup, not donor-only or peak/RSS.
The upstream suite runs natively, not under that focused memory check.

The initial missing-GMP-header compile failed; original source and failed gate
are preserved. Its separate correction adds only the required include prelude
and trailing newline, mechanically verified. No failed numerical fixture was
discarded. Detailed analysis: mpoly-rational-findings.md.

mpoly-rational-experiment.json binds 46 files, ten captured gates (nine successful
and one preserved failed compile), 40 read records, all 956 unchanged live source
hashes, libraries/configuration and both executables. Run here:

```sh
node verify-mpoly-rational.mjs --e-plan-live
```

All 46 checkpoint verifiers pass at 2026-09-10T03:47:27.858Z: exit zero, empty
stderr, 47 records/135,764 bytes. The first 46 records/129,552 bytes exactly match
checkpoint45, including the historical/current binding record. Capture:
results/mpoly-rational-verify-full.{json,stdout,stderr}.

Five continuation improvements remain retained; no new production/donor change,
cleanup, deletion, commit or push. Existing native libraries reused; no Rust/
whole-native rebuild or source copy. Two binaries add 60,840 /tmp bytes; paired
numerical logs add 6,174,888 workspace bytes. About 19GB remains on /tmp.
Continue archived code, generic bridges and recursive support, all remaining
original references and unresolved transfers; reconcile the full inventory.

## Previous checkpoint: finite ARF conversion and integer contracts (45)

The 2,401-fixture finite campaign passes 117,705 assertions against exact GMP
grid rounding and an independent BigInt ordered-bit search/full-integer checker.
It covers 12,005 binary64 outputs, including 2,401 nearest-even cases and 164
midpoints; 11,281 finite imports; 12,005 integer outputs/flags; 7,985 signed-word
conversions called only after their exact values prove they fit; and 14,406
floor/ceil/nint outputs. There are 9,604 public aliases across integer operations
and frexp. Decomposition, integrality, magnitude bounds, signed/absolute power
and previous-value comparisons, and input preservation also pass. Counts repeat
modes/routes. No MPFR output oracle or residue-only comparison is used.

The original harness failed one assertion by expecting zero's signed magnitude
bound to be zero. The donor returns its documented negative sentinel. A separate
mechanically generated v2 skips two unspecified fmpz zero-bound calls, checks the
documented signed result and serializes it as exact decimal text. Every non-bound
output matches v1. Original source, binary and failed gate remain preserved;
no fixture is dropped and no donor defect inferred. Details: arf-conversion-findings.md.

V2 native and Memcheck stdout are identical, 2,781,706 bytes each. Focused
Memcheck passes with zero errors/suppressed/live blocks and all 133,965
allocations freed; 3,847,142 cumulative bytes include the oracle and setup.
This is bounded finite native64 ADX/FE_TONEAREST qualification, not all targets,
hardware rounding environments, upstream tests, donor-only cost or peak/RSS.
No old failure campaign was rerun or discarded.

Source progress adds all 238 lines of fmpz/set.c. Previously covered fmpz/get.c
and ARF rereads add no credit. Combined Calcium/FLINT: 1,064 complete files,
18 partial files and 136,812 unique read lines. The 104-file ARF scope remains
source-complete; recursive support and full ecosystem inventory remain open.

Hyper already separates exact/lossy conversion, uses sticky round-to-odd, and
provides certified ties-away rounding, total near-integer choice and partial
computable-real decisions. No additional production transfer or new benchmark
claim is selected. All 956 live hashes and five retained continuation changes
remain intact. Two preserved executables total 57,336 /tmp bytes; three numerical
logs total 8,340,310 workspace bytes. Existing libraries are reused; no source
copy, Rust/whole-native rebuild, cleanup, deletion, donor edit, commit or push.
About 19GB remains on /tmp.

arf-conversion-experiment.json binds 38 files and eight captured gates: seven
successful and the preserved one-failure original harness. Run here:

```sh
node verify-arf-conversion.mjs --e-plan-live
```

All 45 checkpoint verifiers pass at 2026-09-10T03:17:12.671Z: exit zero, empty
stderr, 46 records/129,552 bytes. The first 45 records/124,861 bytes exactly
match checkpoint44 (including the historical/current binding record).
Capture: results/arf-conversion-verify-full.{json,stdout,stderr}.
Next: signed-add alias and approximate-dot reference pairing, fixed-grid and
remaining scalar contracts as warranted, recursive support, all remaining
original references, unresolved transfers and full inventory reconciliation.

## Previous checkpoint: ARF contracts and test-source closure (44)

Read all 47 remaining ARF test/driver files and the header/manual gaps, 5,783
disjoint lines. All 104 inventoried files in src/arf/, src/arf.h and its manual
are now source-complete. Two supporting random-helper excerpts add 66 lines
and remain partial. Combined Calcium/FLINT coverage is 1,063 complete files,
18 partial files and 136,574 uniquely counted read lines. Recursive support,
other original references and full inventory reconciliation remain open.

Source-level test gaps: main add/sub loops override random modes with truncation;
the signed-add wrapper's limit-one random choice excludes its alias branch;
two binary64 switches have unreachable nearest-even defaults; roots omit
nearest-even and share MPFR with their references. Approximate-dot reference
error pairings use revx for both vectors, even when revy differs. These do not
by themselves prove a library output or memory defect. Original donor files
are unchanged; details are in arf-contract-findings.md and per-file read records.

The new bounded finite native harness checks signed add/sub/div and positive
roots (degrees 1,2,3,5,7,17 plus reciprocal square root), all five rounding modes,
14 precisions from one through 257 bits and supported whole-object aliases.
Exact midpoint neighbors, cancellation, zero numerators and bit/exponent
contrasts are included; denominators stay nonzero. Exact GMP rational scaling,
integer roots and halfway powers verify 518,490 complete values/exactness flags,
316,260 aliases and 64,518 preservation checks. There are 64,860 exact outputs
and 890 repeated ties. Independent BigInt regenerates every fixture, certifies
202,230 primary neighboring/halfway-power bounds and compares every alias in
full, covering all outputs and 103,698 nearest-even checks without an MPFR oracle.

Native/Memcheck outputs are byte-identical, 18,296,286 bytes each. Focused
Memcheck passes with zero errors/suppressed/live blocks; all 1,750,780 allocations
are freed, 37,381,718 cumulative bytes including oracle/setup. Not donor-only
cost, peak RSS, all-FENV/targets/operations or an arbitrary-precision proof.
Nearest-binary64, remaining integer-rounding/comparison and approximate-dot
qualification are still explicit follow-ups. No full upstream ARF suite is
newly executed and older failed evidence remains preserved.

Hyper already has guarded integer/Newton roots, power enclosures, partial
certification boundaries, tested ties-away integer rounding, total multivalued
near-integer choice and round-to-odd dyadic compression. No nonredundant transfer
or new matched benchmark is established. Five continuation improvements remain
retained and all 956 live source hashes unchanged. One new 28,392-byte executable
in /tmp/calcium-arf-contracts.PNgYf4 reuses existing libraries. Paired logs total
36,592,572 bytes in the workspace; no new source copy or Rust/whole-native build.
About 19GB remains on /tmp; no cleanup/deletion, donor patch, commit or push.

arf-contract-experiment.json binds 27 files, five successful captured gates,
51 source-read records, current source/library/configuration and executable
identities. Run here:

```sh
node verify-arf-contracts.mjs --e-plan-live
```

All 44 chained checkpoint verifiers pass at 2026-09-10T02:54:02.872Z: exit zero,
empty stderr, 45 records / 124,861 stdout bytes including the historical/current
snapshot-binding record. The first 44 records / 120,223 bytes are byte-identical
to checkpoint 43. Capture: results/arf-contract-verify-full.{json,stdout,stderr}.
All launched handles are terminal. This is PROGRESS, not ecosystem completion.

## Checkpoint 43: qualified and retained e planner, ARF implementation source closure

The lower-factorial planner is the fifth retained continuation transfer. The
production code is identical to the isolated checkpoint-42 candidate, with
four permanent tests in a 112-line module and four cfg(test) registration lines.
Only three Hyperreal paths change; the other previous source hashes and user
changes are preserved. No new runtime dependency or change to exact series,
splitting, rounding, shared caching or representation.

Baseline/candidate default scalar tests pass 752/756 in both profiles. Candidate
and then retained live all-feature tests pass 859 per profile, with exactly four
added names and no ignored cases. Candidate qualification also passes 24
doctests, strict all-target/all-feature Clippy, formatting, default checks and
fuzz compilation. Baseline/candidate release consumers have identical test
membership: 803 Hypersolve and 1,764 Hypercurve pass, with nine pre-existing
ignored curve cases unrun. Consumer Clippy and supported WASM builds pass.
This is not full CI or every downstream feature/target combination.

WASM execution in local Node22/V8 checks 4,151 exact plans and 300 full integer
outputs per variant using independent BigInt factorial and complete-tail
oracles. All 300 values agree across variants, and all 186 direct kernel values
agree with native checkpoint 42. Fresh module instances and public refinement/
coarsening are included; physical ARM/RISC-V, browser-demo integration, WASM
threads/cancellation/serialization and arbitrary-state claims are excluded.

The targeted native follow-up has ten predeclared groups, 40 alternating blocks
and 1,600 observations. All three earlier slower controls remain included;
the original 71-group evidence is unchanged. Fresh 65,536-bit e takes
3.650 → 1.575 ms, 262,144-bit e 38.035 → 9.073 ms. Deep coarsening remains
slower at paired ratio 1.0271 [1.0235,1.0295], about 59ns/query. Six intervals
improve, one is slower, three overlap. Per-group bootstrap intervals are
unadjusted; selected-group paired ratios are not ratios of aggregate medians.

WASM timing has 22 groups, 12 alternating blocks and 1,056 observations.
Twelve intervals improve, one is slower, nine overlap. Fresh public 65,536-bit
e takes 8.085 → 3.845 ms; 262,144-bit e 82.280 → 31.120 ms. Warm 4,096-bit
queries cost ratio 1.0532 [1.0309,1.0710], about 2.9ns/query more. Cold/uncached
speed and allocation-demand gains justify retaining the small planner despite
these disclosed cached-query costs. Public/kernel peak and live memory are
unchanged; no universal runtime or memory improvement is claimed.

Three unchanged native examples per variant are built, stripped, measured and
run. Hypercurve basic/arrangement shrink 944/928 stripped bytes; Hyperreal
readme_quickstart is byte-identical. The WASM audit module shrinks 507 bytes.
Not whole-Alumina/all-feature/LTO or application-runtime measurements.
Fourteen dedicated executable snapshots total 108,188,255 bytes, plus one
45,446,168-byte source copy in the workspace. The shared offline Rust build
cache is reused; its growth is separate. About 19GB remains on /tmp. No cleanup,
deletion, donor edit, commit, push or external report; old evidence is preserved.

Read 18 further ARF implementation files, 1,176 lines: all 41 inventoried
top-level src/arf/*.c files are now source-complete. Tests, public-header ranges
and recursive MPFR/GMP support remain separate. Nearest-even versus Hyper's
certified ties-away rounding, finite-dyadic versus partial exact-real decisions,
NaN conventions and root domains preclude a direct representation/contract
substitution. No additional transfer or new independent numerical campaign is
claimed for these source reads. Combined Calcium/FLINT coverage is now
1,014 complete / 18 partial files and 130,725 uniquely counted read lines.

`e-qualified-experiment.json` binds 211 files, 55 successful captured gates,
18 full read records, 956 retained source hashes and 14 executable identities.
An initial size-capture tag error and an initial draft read-range correction
are preserved with their original scripts/draft; neither alters numerical
evidence. The corrected draft passes before retention, and all five live gates
pass after retention. All earlier failed numerical, FENV/LLL and thread-memory
evidence remains intact; this is not an all-green history.

Historical verifiers had unconditional reads of old source ranges from the live
tree. After retention, use the explicit snapshot-aware entry point here:

```sh
node verify-e-qualified-retained.mjs --e-plan-live
```

`e-qualified-retention-verification.json` binds the exact historical-path/import
substitutions in 15 versioned modules (82,863 bytes); original files and every
mathematical assertion remain unchanged. All 955 historical hashes and 956
current hashes are checked separately. Full verification passes at
2026-09-10T02:25:08.473Z, exit zero and empty stderr: 44 records / 120,223 bytes
(43 checkpoints plus one source-binding record). Its first 42 records / 112,213
bytes are byte-identical to checkpoint 42. Captured evidence:
results/e-qualified-verify-full.{json,stdout,stderr}.

This is PROGRESS, not completion. Continue unread ARF tests/header and recursive
MPFR/GMP helpers, generic field/matrix/algebraic and every original reference;
full inventory reconciliation and older open experiments remain requirements.

## Checkpoint 42: isolated lower-factorial e planner

Promising, not selected. The candidate changes only e's term planner: a
normalized 64-bit lower factorial mantissa, exact 128-bit word products and
downward truncation cannot stop before the exact threshold. The proof covers
bounded operations/termination over the pre-existing supported precision
arithmetic. Exact coefficients, splitting, rounding and shared caches remain
unchanged. Eight net source lines added; no new donor-source credit. Coverage
remains 996 complete / 18 partial files and 129,549 read lines, not ecosystem
completion.

Per variant: all 4,151 sampled term counts match GMP; all 20,367 steps of the
deepest candidate loop satisfy its lower invariant; 279 directed MPFR numerical
checks and 21 state checks pass. Independent JavaScript BigInt rechecks 8,302
planner answers and 600 numerical enclosures using a separate exact recurrence
and a complete tail bound. Cancellation recovery, serialization, exp(1), shared
refinement/coarsening and four-thread use are sampled. All 855 all-feature
Hyperreal library/integration tests pass in debug/release for both variants,
with identical membership and no ignored tests. Not full CI/downstream coverage.

Four focused sequential Memchecks pass with zero errors/lost/suppressed blocks.
Planner runs retain 544 runtime bytes; numeric runs retain 35,656 runtime/cache/
MPFR bytes, identical across variants. Not zero-live-heap or a thread Memcheck.
Their 8,864 numerical/summary rows match native output. The first build failed
only in the audit harness's Rug conversions/unused import; failed source and
output are preserved. Corrected baseline/candidate builds have no warnings.

Matched CPU: 71 predeclared groups, 3,408 observations, 12 ABBA/BAAB blocks,
CPU 6, paired bootstrap intervals without multiplicity adjustment. All ten
planner and ten kernel intervals improve; overall 37 below one / three above /
31 overlapping. Fresh public 65,536-bit e: 3.405 to 1.294 ms; requests 11,558 to
4,643; requested bytes 32,693,112 to 1,628,872. Public peak/live demand unchanged.
The separate 426 allocation observations have fewer requests/bytes in 28 groups
and no increases; only seven planner-only peaks decrease. Cached/coarsened and
unchanged pi controls have mixed timings, including slower intervals. No blanket
runtime, memory or size benefit is asserted.

Six frozen executables total 11,204,552 bytes in /tmp/calcium-e-plan.mOIsBs.
CPU/allocation audit files shrink 1,304/1,472 bytes, the oracle grows 944; these
are not representative consumer sizes. Only 180 Hyperreal files were copied
(5,312,930 original bytes); the existing Rust build cache is reused. About 21 GB
remains on /tmp. No production/donor edit, cleanup, deletion, commit or push.

`e-plan-experiment.json` binds 94 files, 21 gates (20 successful and the preserved
initial build failure), 955 unchanged live hashes, 180 candidate hashes and six
frozen executables. Its historical command was
`node verify-e-plan.mjs --derivative-live`; after checkpoint-43 retention use
the snapshot-aware entry point above. At checkpoint 42, work before
retention: durable regressions, default-feature/downstream checks, Clippy/fuzz/
WASM, other-target u128 costs and representative sizes. Continue unread support,
all original references and inventory reconciliation; all older gaps remain.

All 42 chained verifiers pass at 2026-09-10T00:50:00.566Z: 42 JSON records /
112,213 stdout bytes, empty stderr, exit zero. Preserved capture:
results/e-plan-verify-full.{json,stdout,stderr}. After report updates, all 94
bound files, 955 live hashes, 180 candidate hashes and six executable hashes
still match. No active process is left by this checkpoint.

## Checkpoint 41: magnitude combinatorial/tail/conversion bounds

This pass adds 3,478 disjoint lines: 28 remaining mag tests/driver, full ARF I/O
and the remaining ARF conversion ranges. All 104 inventoried files under
src/mag/ are now source-complete, plus the already-read public header/manual.
Recursive support and numerical qualification remain separate. Calcium/FLINT
coverage is 996 complete / 18 partial files, 129,549 lines—not ecosystem completion.

Native checks pass 48,890 exact comparisons, 3,358 directed comparisons,
1,441 input checks, 1,240 supported alias agreements and 402 valid string
roundtrips. JavaScript BigInt independently rechecks 48,086 rational bounds
and 201 double exports, including a different Bernoulli defining recurrence.
Native 603 exact rational/ceil/floor comparisons are not separately emitted for
JS recomputation. Factorials sweep 0..4096; binomials cover 36,237 inputs.

Full-tail references use exact rational geometric sums, eventual-geometric
remainders for polylog and zeta-minus-prefix for Hurwitz. Each MPFR precision
(512/768) covers 576 polylog and 527 Hurwitz cases, with identical outputs.
Of the convergent polylog cases, 199 return conservative infinity and 377 finite
bounds. The 796 repeated infinity outputs are included in the directed count;
they are valid but not useful finite-enclosure results.

Native and Memcheck pass with identical 1,308,580-byte numerical output. Memory:
zero errors/live blocks, 636,551 allocations/frees and 73,880,835 cumulative bytes
including oracle work—not peak or donor-only cost. One 38,448-byte executable
in /tmp/calcium-mag-series.o08QL0 reuses libraries. No full rebuild or cleanup.

No production transfer. Hyper already keeps exact combinatorial values and
structural cancellation. A planning-only e term-count candidate is identified:
a fixed-word lower factorial enclosure might avoid building the first growing
BigInt before binary splitting. It remains unimplemented and unselected pending
proof, independent numerical/state checks and matched CPU/allocation/size gates.

`mag-series-experiment.json` binds 26 files, six successful gates, 30 read records,
955 unchanged live hashes, libraries/configuration and the executable. Run
`node verify-mag-series.mjs --derivative-live` here. All 41 chained verifiers pass
at 2026-09-10T00:10:30.054Z: 41 JSON records / 106,842 stdout bytes, empty stderr,
exit zero. Earlier failures, snapshots and crash evidence remain preserved.

Next isolate the planning candidate and continue ARF/scalar/MPFR support,
generic field/matrix/algebraic and original references. Full inventory
reconciliation remains open. No donor/production edit, deletion, commit, push,
external report or unsafe/malformed-input reproduction in this pass.

## Checkpoint 40: magnitude transcendental/root/tail bounds

No new production transfer. This pass reads 44 complete files and 4,660 new
lines: all 29 remaining top-level mag C implementations, two headers and thirteen
donor tests. All 52 top-level implementations are source-complete, not all
tests/profiles or recursive dependencies. Calcium/FLINT coverage is 966 complete /
19 partial files and 126,071 lines; full ecosystem completion remains open.

The bounded corpus passes 36,208 directed-bound checks, 31,652 supplemental
quality checks and 16,742 checks each for input preservation and supported whole
aliases. Per reference precision: 371 unary inputs, 16 elementary functions,
eight root degrees, 99 exp-tail cases, 680 positive double-log inputs and both
pi bounds. Direct exact dyadic MPFR imports avoid ARF conversion and enormous
integer expansion. Two precisions (512/768 bits) yield identical donor outputs;
they are not separate implementations or a formal proof.

The donor exp-tail test checks only 50 terms. The new positive-series oracle
also bounds every omitted term geometrically. This establishes full-tail bounds
for the tested x <= 4 and N <= 257, not all inputs. Other-tail, combinatorial,
conversion/I/O, promoted/nonfinite, arbitrary-state, all-FENV and architecture
qualification remains open. Source-only I/O concerns were not reproduced.

Native and Memcheck pass with identical 866,255-byte numerical output. Memory:
zero errors and no live heap blocks, 116,716 allocations/frees, 22,125,312
cumulative bytes including the oracle—not donor-only demand or peak memory.
One 33,256-byte executable in /tmp/calcium-mag-transcendental.p8Wy83 reuses
existing libraries. No native-tree or Rust rebuild, cleanup or deletion.

Hyper already certifies series domains, budgets truncation/precision, reuses
exact-rational reduction and uses integer root enclosures. No nonredundant
transfer or matched performance benefit is established. Four retained
continuation changes remain unchanged; prior failed evidence remains preserved.

`mag-transcendental-experiment.json` binds 26 files, six successful gates,
44 read records, 955 live hashes, libraries/configuration and the executable.
Run `node verify-mag-transcendental.mjs --derivative-live` here. All 40 chained
verifiers pass at 2026-09-09T23:47:37.528Z: 40 JSON records / 100,963 stdout
bytes, empty stderr, exit zero. This checks evidence integrity, not universal
correctness or successful reclassification of earlier failed numerical gates.

Next qualify source-read combinatorial/conversion/other-tail code as warranted;
finish remaining mag tests/profiles, ARF conversion/scalar/MPFR support, then
remaining generic field/matrix/algebraic and original ecosystem references.
Full inventory reconciliation stays open. No donor/production edit, commit,
push or external report in this pass.

## Checkpoint 39: magnitude bounds/error inflation

No production transfer. This pass adds 36 new full-file reads, completes the
formerly partial magnitude header, and adds three other partial records:
3,978 uniquely counted lines. Coverage is now 922 complete / 19 partial files,
121,411 lines across Calcium/FLINT—not the complete ecosystem inventory.

The independent rational corpus passes 505,048 magnitude-direction checks and
the same number of supplemental relative-quality checks, 14,688 comparisons,
31,416 magnitude/conversion input checks, 6,480 Arb FMA/error-inflation results
(12,960 endpoint comparisons) and 1,584 ball input/midpoint checks. Exact squaring
checks root bounds without a root oracle. There are 7,344 magnitude pairs, 680
constructor fixtures through 513 bits, and 144 ball fixtures through 331 limbs.
Counts include repeated routes, signs, precisions and supported aliases.

Upper/lower bounds are not correctly rounded values. Decreasing operands need
opposite polarity, and finite fast paths also constrain destination exponents.
Hyper already separates structural facts from approximate magnitude planning;
no new representation change or matched benchmark benefit is established.
Direct hypot, large/promoted powers, nonfinite/extreme exponents, all libm/
architecture/thread environments and remaining transcendental/tail/IO code
remain unqualified. GMP shares some FLINT integer support; no formal proof or
second independent integer backend is claimed.

Native and Memcheck pass with identical 197,917-byte numerical output. Memory:
zero errors and no live blocks, 9,839 allocations/frees, 6,879,352 cumulative
bytes including setup/oracle work—not peak RSS or donor-only costs. The existing
native build is reused; one 47,216-byte executable is preserved in
/tmp/calcium-magnitude.ipqBZw. Its unused previous harness main contributes to
the audit binary, so this is not a production size measurement.

`magnitude-experiment.json` binds 35 files, eight gates (six successful and two
preserved corrected audit failures), 40 read records, 955 unchanged live hashes,
libraries/configuration and the executable. Initial C formatting warnings and
the initial expected-line-count error remain preserved with their source
snapshots. Their corrections changed no non-whitespace C tokens or read ranges.
Run `node verify-magnitude.mjs --derivative-live` here. All 39 chained verifiers
pass at 2026-09-09T23:12:01.924Z: 39 JSON records / 95,243 stdout bytes, empty
stderr, exit zero. Earlier mathematical/memory failures remain failures.

Next continue remaining magnitude functions, unread ARF conversions and scalar
support, including external MPFR high-helper source, then generic field/matrix/
algebraic and every original reference. Full inventory reconciliation remains
open. No cleanup, deletion, donor edit, commit, push or external report.

## Checkpoint 38: Arb dot/error-bound support

No new production transfer. This pass adds 20 completely read files and three
partial reads, 3,327 new lines: all ball/typed-integer dot implementations,
error-inflation and fused support, relevant donor tests, declarations and public
stride/alias contracts. Cumulative coverage is 885 complete / 19 partial files,
117,433 lines; this is not completion of the ecosystem inventory.

An independent GMP rational endpoint oracle checks 198,288 results / 396,576
endpoint comparisons across 648 fixtures and 1,224 operation groups. The three
ball routes each have 46,656 calls; each of five integer wrappers has 11,664.
All 21,349 input checks also pass. The bounded corpus spans twelve ball widths
through 333 limbs, six exact/uncertain/cancellation/sparse families, four lengths,
three valid stride layouts, both signs, three initial-value/alias modes and nine
precision positions. Typed wrappers use three widths and integer coefficients
through 258 bits. Counts include repeated routes/precision positions, not a
formal proof or instrumented branch-coverage guarantee.

Native and Memcheck output matches exactly (161,471 bytes each). Memory reports
zero errors and zero live blocks, 687,105 allocations/frees and 727,232,808
cumulative bytes including setup/oracle work. GMP is separate from Arb's
enclosure algorithm but also underlies some FLINT integer work. No second
integer-backend, tightest-enclosure, correct-rounding or peak-RSS claim.

Keep truncation, propagated uncertainty and final rounding as separate certified
error terms. Integer adapters' shallow views and input-uncertainty precision
caps are useful patterns, but Hyper already carries exact factors into shared-
scale/dyadic reducers with delayed reduction. Truncated midpoint products are
not substitutes for exact-value results. No nonredundant transfer with measured
benefit was established; no matched Hyper benchmark or production change here.

`arb-dot-experiment.json` binds 27 files, six successful gates, 23 read records,
955 unchanged live hashes, five shared library identities, native configuration
and one 27,768-byte executable in /tmp/calcium-arb-dot.zsWvwv. Run
`node verify-arb-dot.mjs --derivative-live` here. Full verification passes at
2026-09-09T22:45:24.102Z: 38 JSON records / 88,516 stdout bytes, empty stderr,
exit zero. Failed historical gates, binaries, source snapshots and crash
evidence remain intact; no full native rebuild or cleanup was needed.

Direct FMA/add-error APIs, exceptional radius-only midpoint, huge/nonfinite,
self-dot correlation, negative-length, arbitrary alias/thread and other native
architecture cases remain unqualified. MPFR high-helper source is an explicit
dependency gap, not conflated with earlier FLINT high-product bound failures.
Continue remaining Arb/magnitude/scalar support, generic field/matrix/algebraic
support and all original references, with full inventory reconciliation open.

## Checkpoint 37: complex-product v2 crossover/dispatch/lifetime

V2 also remains **unselected**. It raises the experimental minimum to 256 bits,
uses the existing word scan for early bypass, keeps the helper out of line and
shortens signed-sum lifetimes. Its one-file isolated edit adds 78 lines over
retained baseline; no live or donor source changes.

The unchanged release GMP oracle passes 82,944 component checks / 6,912 fixtures
and 27,648 input checks, including the crossover boundaries. Native/Memcheck
stdout matches frozen baseline. Memory diagnostics have zero errors/lost blocks
and the same 13,832 reachable bytes in 136 blocks. A fresh candidate trace passes
5,832 rows through 65,536 bits against exact num-rational arithmetic, selects the
new path 1,800 times and preserves public first-use/reuse labels. The large-input
comparison shares the Rust integer backend; baseline trace/memory evidence is reused.

Fresh paired timings and allocations rerun both variants. CPU: 9,216 ABBA
observations / 26,810,928 queries over the same 192 groups. Ratios 0.4962–1.2642;
66 per-group intervals below parity, 17 above, 109 overlapping. Of 60 selected
groups, 55 favor v2 and none has an interval wholly above parity. All 17
above-parity intervals are bypass/reused controls. Intervals are not
multiplicity-adjusted; no universal effect or direct paired v1/v2 comparison.

Allocation: 1,152 observations. Requests fall in 57 groups and requested bytes
in 60, neither rises; live deltas match. Peak falls in 27 groups but rises in 33.
For the same 60 selected groups, request demand equals v1 and peak is lower in
36. Unstripped CPU/allocation files grow 9,192/9,152 bytes over baseline; these
are benchmark, not representative application sizes. No exactness/completeness
gain justifies accepting all these costs as a general dispatch replacement.

`complex-product-v2-experiment.json` binds 68 files, 12 new successful gates,
three reused baseline gates, both 955-file source maps and eight binaries. Four
new executables total 8,367,856 bytes; four existing baseline executables are
reused. Run `node verify-complex-product-v2.mjs --derivative-live` here. All
37 chained verifiers pass: 37 JSON stdout lines / 82,500 bytes, empty stderr,
exit zero at 2026-09-09T22:17:02.172Z. Earlier failed gates remain preserved.
No new donor read credit: 865 complete / 16 partial files, 114,106 lines.

Both prototypes and all evidence remain intact. No new full crate/debug/state/
concurrency/quotient-cost/WASM/application-size qualification or fifth retained
transfer. Next resume Arb dot/error-bound support and the remaining original
references. Revisit common-scale multiplication only with relevant application
workload evidence, not another speculative generic dispatch variant.

## Checkpoint 36: cold wide-rational complex-product v1

The three-product idea is promising, but this implementation is **not selected**.
It changes only an isolated Hyperreal snapshot: a common per-operand denominator
and shape-cost gate select three exact signed integer numerator products after
the existing word path. Ordinary and conjugate products use the existing final
reduction and multiplication machinery. No live or donor source changes.

Both variants pass 82,944 exact component checks over 6,912 fixtures, plus 27,648
input-preservation checks. The independent GMP rational oracle covers 16 widths
from 32 through 2,048 bits, nine shape/cancellation families, three scales, all
16 sign masks, direct multiply/divide and public Hyperlattice multiply, each
called twice. Counts include related routes and repeated calls. Memcheck outputs
match native outputs, with zero errors/lost blocks and the same 13,832 bytes
reachable in 136 blocks. Cumulative allocation totals include the oracle.

Separate traces cover 5,832 rows per variant through 65,536 bits, checked against
exact num-rational arithmetic (sharing the Rust integer backend). The new scalar
selector is reached 2,520 times. Public first-use/reuse labels match between
variants; repeated Hyperlattice calls retain the existing reuse schedule.

Uninstrumented CPU: 9,216 ABBA observations / 26,799,888 queries in 192 groups,
six widths through 16,384 bits, eight scenarios, scalar/public multiplication,
first-use/prewarmed inputs. Construction, pool lifetime and oracle work are
excluded. Ratios span 0.5043–1.4353: 64 unadjusted per-group intervals favor the
candidate, 43 favor baseline and 85 overlap parity. Of 75 selector-reaching
groups, 59 improve, nine regress (all 192-bit cases), and seven overlap.
Bypass/cached controls also show costs. This is not a quotient timing campaign.

Separate allocation measurements: requests fall in 72 groups and requested
bytes in 75, with no increases; live deltas agree. Peak demand rises in 66 groups,
falls in nine and agrees in 117. Unstripped CPU/allocation executables grow
8,864/8,792 bytes; these are not representative application-size measurements.
A selected 16,384-bit first-use public multiply improves from 109.536 to
81.903 microseconds, while its measured peak rises from 21,640 to 33,680 bytes.
The current shape gate therefore does not justify general retention.

`complex-product-experiment.json` binds 71 files, 16 successful gates, both
955-file source maps, unchanged retained live hashes and eight executables
totaling 16,697,200 bytes. Run `node verify-complex-product.mjs --derivative-live`
here. All 36 chained verifiers pass: 36 JSON lines / 78,703 stdout bytes, empty
stderr, exit zero at 2026-09-09T21:52:51.195Z. Previous failed mathematical and
memory gates remain preserved. No new donor coverage: 865 complete / 16 partial
files, 114,106 lines.
The candidate source copy is 45,445,675 bytes before its one-file edit; the
baseline and shared Rust/native build caches are reused. Prior evidence remains
preserved, with no cleanup, deletion, commit or push.

Next: a separate revision investigating crossover, dispatch overhead and shorter
temporary lifetimes, then matched costs and broader qualification if worthwhile.
No new full crate/debug/state-history/concurrency/internal-unreduced-storage/
WASM or application-size qualification is claimed. Full ecosystem scope remains
open; this is not a fifth retained continuation transfer.

## Checkpoint 35: ARF fused/complex cancellation

All 829,440 scalar-component/value/exactness checks pass, plus 19,584 input
checks. The corpus has 576 fixtures: 12 base lengths through 251 limbs,
12 cancellation/zero/gap/imbalance families, four selected sign masks (not
all 16), 12 precision positions and five modes. Twenty-four result routes
cover separate/fallback/left-/right-in-place complex multiplication, ordinary/
in-place complex square, three FMA storage forms, addmul/submul, sum-of-squares,
addition/subtraction, four-term forward/reverse sums and two-term exact dots.
Complex operations count two component results per call, and repeated arithmetic
is not described as independent mathematical identities.

The unchanged checkpoint-34 GMP oracle compares every full decoded value and
flag after forming the complete integer expression. Its previous main is
compiled but not executed. Independent BigInt reconstruction checks all 13,824
groups and 345,600 expression roundings, including membership/order, decision
counts and compact fingerprints. Fingerprints supplement the full GMP value
comparisons; they are not collision-free certificates. The 468 zero and 600
unit-magnitude integer reference positions include repetitions/zero controls
and are not necessarily real values of magnitude one after dyadic scaling.

Native/Memcheck numerical output matches (2,176,528 bytes each), both exit zero,
and Memcheck reports zero errors/live blocks. All 2,249,297 allocations are
freed; cumulative 3,530,423,920 bytes include setup/oracles/product vectors,
not peak RSS or donor-only cost. No invalid sizes/raw overlap, extreme exponent,
nonfinite sweep, arbitrary thread histories, all strides/lengths/aliases,
approximate-dot/high-complex, ARM/32-bit/FFT or whole-stack qualification.

Twenty complete files and two header/docs extensions add 4,131 read lines:
ARF addition/fused/sum/dot/memory support, donor tests, Gaussian-integer square,
exact shifted limb import and all 1,014 lines of complex limb kernels. The
combined continuation coverage is 865 complete/16 partial files, 114,106 lines.
FFT-conditioned source is read but disabled in this native build. The optional
mantissa free-list cache is also disabled; addition's TLS scratch is separate.

No production/donor edit or fifth continuation transfer. All 955 live hashes
remain unchanged. Current cold wide-rational complex multiplication still has
a four-product fallback after the word-sized path, while the donor can select
three products by shape and reduce scratch/copies. This is a concrete next
isolated benchmark candidate, not a measured improvement. Preserve existing
small/unbalanced/cache-sensitive routes and qualify exactness, state, CPU,
allocation and representative size before retention.

`arf-fused-experiment.json` binds 31 support/evidence files, seven successful
gates, 22 read records, 955 live hashes, native/configuration hashes and one
32,008-byte executable. Run `node verify-arf-fused.mjs --derivative-live` here.
The recorded full chain passes all 35 checkpoints: 35 JSON stdout lines /
75,395 bytes, empty stderr, exit zero. Previous failures remain preserved.
Existing native/Rust builds and all earlier failed/successful evidence are
preserved. No cleanup, deletion, commit, push or external report. Remaining
ARF/Arb-dot/generic matrix/field support and all original references remain
in scope; full inventory reconciliation is not complete.

## Checkpoint 34: specialised high products and ARF rounding

No additional Hyper or donor change is selected. All 955 retained live
source/support hashes still match checkpoint 33. Hyper already has demand-sized
constructive approximation planning, shared square work and exact integer
products before final scaling. Hyperlattice already has evidence-gated
three-product complex multiplication, fused exact-rational products, sparse
product sums and known-exact shared-scale dot dispatch. The donor's tuned
limb thresholds do not establish a better choice for these workloads.

Independent GMP integer products and quotient/remainder rounding check 612,000
finite ARF outputs and exactness flags, plus 2,880 input checks. Ten routes
cover set/neg rounding with separate and supported in-place output, public/
swapped/in-place multiplication, explicit MPFR multiplication, public square
and MPFR square. There are 18 length pairs through 1,001 limbs, ten patterns,
four sign forms, 17 precision positions and five modes; repeated/related
arithmetic is explicitly included in these counts. MPFR is not the oracle.
Output decoding reads normalized limbs directly, without ARF conversion or
rounding helpers.

JavaScript BigInt separately reconstructs 244,800 arithmetic roundings and
checks membership, order, counts and compact fingerprints of all 7,200 output
groups. Fingerprints are not collision-free certificates; full equality is
checked by the GMP oracle. Native and Memcheck stdout are byte-identical
(1,142,532 bytes each), with both exits zero. Memcheck reports zero errors/live
blocks and all 303,090 allocations freed; 976,772,600 cumulative bytes include
test setup/oracle work, not peak RSS or donor-only allocation cost.

Twenty complete source files plus two partial header/docs records add 7,570
unique read lines. Combined continuation coverage is 845 complete/16 partial
files, 109,975 lines. Includes the seven hardcoded/normalised/ARM high-product
assembly files, ARF rounding/special/fused/complex implementations, donor
rounding tests and contracts. The ARM assembly is read-only qualification;
no ARM execution or instruction-level proof. No new fused/complex numerical,
large-exponent, arbitrary thread, infinity/NaN, 32-bit or FFT qualification.
The earlier 24 integer / 44 full-residual high-product failures remain intact;
passing ARF controls does not erase a different low-level contract discrepancy.

`arf-rounding-experiment-v2.json` binds 36 evidence/support files, eight gates
(seven successes plus a preserved metadata-verification failure), 22 read
records, 955 live hashes, native/configuration hashes and one 22,856-byte
executable. The first manifest recorded a Hyperlattice read through 255 when
the file ends at 250. The verifier caught it after all 33 earlier checkpoints;
the initial manifest, verifier and failed run are preserved unchanged. V2
corrects only that read-range metadata, without altering numerical or source
evidence. Run `node verify-arf-rounding-v2.mjs --derivative-live` from this
directory. The corrected recorded run passes all 34 checkpoints: 34 JSON lines /
67,196 stdout bytes, empty stderr, exit zero. No new whole-tree build, deletion,
commit or push.
Next: finite fused/complex cancellation qualification as needed, remaining
ARF/addition/temp/generic matrix/field support and all remaining original
references. Full ecosystem scope and inventory reconciliation remain open.

## Checkpoint 33: independent high-product bounds

The documented n+2 guard-limb error bound is too small for some valid inputs.
Independent GMP full products check 53,248 outputs from 9,216 input cases,
144 lengths through 2,049 limbs and 32 deterministic patterns. Public multiply,
square, caller-scratch multiply, internal naive/recursive (up to 128 limbs),
and normalised multiply/square are covered. Outputs are disjoint; normalised
routes receive top-bit-set inputs. Native build is 64-bit ADX, FFT-small off.

There are 24 integer-bound failures: twelve public square/normalised-square
outputs at four lengths and twelve internal multiplication-reference outputs
at three lengths. Twenty further rows fail only the full fractional-residual
interpretation. Public square examples have deficit 30 versus bound 25 at n=23,
and 124 versus 89 at n=87. Repeated routes/forms are not distinct defects.
All outputs remain one-sided and within the looser scaled 2n bound; all 832
public full-product-fallback checks are exact. No Hyper/nfloat wrong result
or arbitrary-precision error proof follows from this corpus.

JavaScript BigInt independently reconstructs 3,059 output rows, including all
44 discrepancies, from full products and the exact omitted triangular tail.
Native/Memcheck numerical logs match byte-for-byte. Memcheck reports zero
errors/live blocks and all 628,419 allocations freed; cumulative bytes include
the oracle. Both processes exit 1 because mathematics checks fail: these are
preserved failed numerical gates, not clean overall qualification.

Seven complete files and two header extensions add 1,708 read lines. Effective
continuation coverage is 825 complete/15 partial files, 102,405 unique lines.
Read ADX basecase multiply and odd/even square, ARF multiplication/MPFR bridge/
scratch cleanup, and associated macros. Hardcoded/normalised/arm/FFT support
and rounding helpers remain open. Hyper already plans precision and exactly
multiplies integer approximations before scaling. No production transfer,
matched performance claim or donor cutoff adoption is justified here.

`high-product-experiment.json` binds 30 support/evidence files, seven gates
(five successes, two numerical failures), nine read records, the unchanged
955-file live snapshot, native/configuration hashes and one binary. An initial
binding draft is separately preserved after correcting an agent-file read-range
typo before verification. Run `node verify-high-product.mjs --derivative-live`
from this directory for 33 chained verifiers. Recorded run: 33 JSON lines /
60,998 bytes, empty stderr, exit 0. Passing verification preserves failures;
it does not declare every numerical/memory gate successful.

New `/tmp` executable: 22,616 bytes; existing library/build cache reused,
about 21 GB free. No production/donor change, deletion, commit or push.
Next: remaining high-product and ARF support, independent rounded-product
controls where relevant, then generic matrix/field support and the remaining
original references. Full ecosystem scope remains incomplete.

## Checkpoint 32: bit-mask filter and high-product reads

Neither sign-filter implementation is retained. The mask preserves the same
small-input path and ordered Unknown/trace behavior; both 956-file source maps
are verified against the frozen baseline. The retained live 955-file map remains
unchanged. Candidate debug/release each pass the same 364 tests, including
299,593 exhaustive short sequences, 1,280 long cases and 32 private traces.
All 45 public traces match the baseline. The prior 24-ring independent rational
shoelace/Machin oracle is reused.

The mask CPU campaign records 2,304 batches / 8,346,816 repeated queries in
48 groups, 12 ABBA blocks/group on CPU 6. Ratios span 0.964811..1.139742;
two per-group bootstrap intervals below one, seven above, 39 overlapping.
Intervals are not multiplicity-adjusted and bypass controls also vary. Separate
288 allocation batches exactly match the first prototype's measurements:
24 reaching groups save one request and 6..256 bytes per query, 24 bypass
groups are unchanged; 12 groups improve peak bytes, all live deltas are zero.
Unstripped CPU/allocation driver files shrink 1,176/1,264 bytes and the trace
driver shrinks 2,304 bytes. These artifact-specific savings do not outweigh the
mixed runtime evidence with no exactness/completeness gain. No universal speed,
memory-bound, RSS or downstream binary-size claim; no new sanitizer/WASM gate.

Twelve high-product implementation/test files and two partial header/docs add
2,196 read lines. Effective continuation coverage is now 818 complete/15 partial
files, 100,697 unique lines. A guard limb does not make a high product exact:
omitted products can carry. Documentation promises n+2 guard ulps; larger donor
tests allow 2n, while smaller/normalised tests use sibling consistency. No new
numerical defect or independently qualified high-product accuracy is claimed.
Donor tests were read, not newly executed. Assembly/FFT callees remain open.

`sign-filter-mask-experiment.json` binds 41 support/evidence files, eight
successful terminal gates, source maps, 14 read records and six executable
snapshots. Run `node verify-sign-filter-mask.mjs --derivative-live` from this
directory for all 32 chained verifiers. The recorded run passes with 32 JSON
lines/55,226 stdout bytes, empty stderr and exit 0. Three new snapshots total
6,349,368 bytes; baseline binaries and the shared target are reused. About
21 GB /tmp remains, not a quota or cache-growth guarantee. No production/donor
change, deletion, commit or push; all earlier failed evidence is preserved.

Next: independent GMP full-product checks on bounded valid inputs, then called
high-product/ARF/generic matrix/field support and the remaining original
references. The full ecosystem audit is incomplete.

## Checkpoint 31: two-sign constant-state filter, unselected

The isolated >4-term filter keeps at most two distinct nonzero signs, preserving
the old small-input path and ordered Unknown/trace behavior. Both 956-file
snapshots contain identical 125-line tests; candidate function delta is +13/-3.
The retained live 955-file snapshot is unchanged. No new donor read coverage:
806 complete/13 partial files, 98,501 unique continuation lines.

All 299,593 short-sequence and 1,280 long-sequence cases pass; 32 private trace
cases match the reference. Baseline and candidate each pass the same 364 tests
in 16 suites, debug/release, no failures/ignored. Forty-five public query traces
match both variants and checkpoint 30. A separate exact-rational shoelace and
Machin alternating-series oracle proves signs for all 24 benchmark ring cases.

CPU campaign: 48 groups, 2,304 measured batches, 8,884,320 repeated queries,
12 ABBA blocks/group on CPU 6. Ratios span 0.946269..1.080495; five per-group
bootstrap intervals below one, eleven above, 32 straddling. Not multiplicity-
adjusted and not a universal performance result. Separate 288 allocation batches
save one request and 6..256 bytes per query in all 24 filter-reaching groups;
24 bypass groups are identical. Peak live bytes improve in 12 groups, equal
in 36; all post-batch live deltas zero. Not RSS or a general retention bound.

CPU/allocation benchmark files grow 2,536/2,552 bytes (unstripped); trace build
shrinks 648 bytes. These feature-specific artifacts do not measure downstream
application size or establish a codegen cause. This first implementation stays
UNSELECTED: no exactness/completeness gain and mixed runtime/size costs for a
small allocation saving. Next test a leaner bit mask without weakening semantics.

`sign-filter-experiment.json` binds 67 evidence/support files, 15 successful
terminal gates, source maps and six binaries. Run
`node verify-sign-filter.mjs --derivative-live` for 31 chained verifiers. Recorded
run passes with 31 JSON lines/51,122 bytes, empty stderr and exit 0. Dedicated
snapshots total 12,712,664 bytes; shared target reused; about 21 GB /tmp remains.
No new sanitizer/WASM/concurrency/abort/serialization/downstream qualification
is claimed for this unselected trial. No production change, deletion, commit
or push. The full ecosystem audit remains open.

## Checkpoint 30: complex nfloat closure and filter reachability

Six completed files add 2,976 lines; all 26 tracked files under src/nfloat are
now read. Effective continuation coverage is 806 complete/13 partial files,
98,501 unique lines. Called supporting kernels and the full ecosystem remain
open. The four retained continuation transfers are unchanged.

The approximate complex layer uses guard limbs, four/three-product dispatch,
axis shortcuts and cancellation-aware principal roots. Hyperlattice already
has reuse-sensitive exact-rational three-product multiplication and fused cold
kernels. Neither donor limb cutoffs nor midpoint-as-value conversions justify
changing exact scalar contracts. Sequential mutating vector profilers do not
establish matched exact-real performance.

Independent finite controls pass 89,628 rows: 87,516 arithmetic/2,112 exact
squared-norm comparison cases, 177,144 rational comparisons and 58,344
output-placement equality checks (37,488 actual input aliases; 20,856 unary
outputs use an otherwise-unused operand buffer). All 66 word precisions and
32 deterministic patterns are covered;
zero reciprocal/division cases are skipped. Accuracy is a stated finite-corpus
tolerance, with root squared residuals and branch signs, not a general rounding
or enclosure proof. Native/Memcheck numerical logs match. Memcheck has zero
errors/live blocks and all 2,601,723 allocations freed; allocation totals include
the oracle. Six unchanged Hyperlattice complex tests pass in debug/release.

The public strict near-pi ring-area probe reaches the >4-term sign filter in
all 15 near-cancellation queries; 30 rational/well-separated controls bypass it.
All return exact Positive at the expected stages. This is reachability only,
not a candidate implementation, cold-process benchmark or retention decision.
Next isolate the constant-state filter while preserving Unknown and trace order,
then qualify matched CPU/allocations/state/retention/binary costs.

`nfloat-complex-experiment.json` binds 49 files, 12 gates (ten successes and two
preserved initial compile failures), six read records, two unchanged 955-file
source maps and two binaries. Run `node verify-nfloat-complex.mjs --derivative-live`
for all 30 chained verifiers. Recorded run passes with 30 JSON lines/46,512
bytes, empty stderr and exit 0. Dedicated snapshots total 2,242,256 bytes;
existing builds are reused and about 22 GB /tmp remains. Snapshot bytes do not
measure cache growth. No production/donor change, deletion, commit or push.
Earlier numerical, memory, instrumentation and assertion evidence is preserved.

## Checkpoint 29: fixed-point kernels and bounds

Thirteen additional implementation/test/profiler files are fully read, adding
3,286 lines. Effective continuation coverage is 800 complete/13 partial files,
95,525 unique lines. Full source inventories and the original ecosystem scope
remain open; the previous four retained transfers are unchanged.

An exact native-vector witness establishes that a Strassen intermediate can
reach 4A while the range-bound routine uses 3A. The witness is only 2^-18 and
fails the bound in 24 repeated precision/FENV configurations, not 24 independent
bugs. Another 52 of 222 automatic/explicit-cutoff bound pairs differ, matching
duplicated dispatch/parity logic. These are not observed wrong final products.

Independent exact-integer checks find no output-error-bound failures in 560
dots and 1,740 matrices: 1,922,900 comparisons per run across classical,
Waksman, automatic/forced Strassen and public fixed-point dispatch. Carefully
scaled native 64-bit positive-dimension inputs do not qualify overflow, raw
empty dots, aliases, arbitrary concurrency, 32-bit or complex arithmetic.
Memcheck reports zero errors and all allocations freed, but the driver exits
1 for the intermediate-bound failures. Its allocations include the exact oracle.

Native/Memcheck outcomes and reported maximum errors agree; 908 rows differ in
1,225 binary64 bound fields. Each run is independently validated. A no-FLINT
volatile-product/MPFR control agrees in all 12 native cases and differs in four
under Memcheck while reported FENV modes remain unchanged. Both failures and
the initial overstrict log-equality checker are preserved, not suppressed.
This is a local instrumentation qualification, not general donor validation.

No production or donor change is justified. Hyper's exact filters and explicit
uncertainty remain appropriate; existing two same/mixed-sign tests pass in
debug/release. A constant-state replacement for the >4-term sign filter's Vec
is a future candidate only; the existing rational ring benchmark bypasses it.

`nfixed-experiment.json` binds 47 evidence/support files, 12 terminal gates
(seven successes/five preserved failures), 13 read records, unchanged 955-file
source maps, two binaries and libraries. Run
`node verify-nfixed.mjs --derivative-live` for 29 chained verifiers.
Recorded run passes with 29 JSON
lines/43,065 bytes, empty stderr and exit 0. Inventories and effective coverage
verify. New /tmp executables total 40,312 bytes; existing builds are reused and
about 23 GB remains available, not a quota guarantee. No deletion, donor patch,
commit or push. Earlier field-relation assertion evidence remains unresolved.

## Checkpoint 28: nfloat arithmetic/conversion support

Completed the remaining scalar implementation, main arithmetic tests, manual,
dot/matrix implementations and generic classical matrix multiplier; read the
first 95 lines of the generic matrix header. This adds 7,568 unique lines and
six completed files. Cumulative continuation coverage is 787 complete/13 partial
files and 92,239 lines. Full inventories and original ecosystem scope stay open.

Independent GMP-rational controls pass 44,574 native cases/74,922 comparisons:
31,680 scalar operations, 5,280 conversions, 6,912 forward/reverse dots and
702 matrices/31,050 entries. Both directed modes, all 66 native word precisions
for scalar/conversion operations, nine dot/matrix precisions, whole aliases,
sparse/cancelling/exponent-gap inputs and distributed caller FENV modes are
included. Directional inequalities are not correctly rounded equality claims.
Memcheck output is identical, with zero errors and all 595,961 allocations freed.

The documented directed subset excludes parsing, transcendental functions,
most mixed operations and complex arithmetic. Public directed real matrix
multiplication selects classical evaluation; fixed/block routes are unqualified
by these controls. Finite native 64-bit cases do not qualify exponent extremes,
underflow flushing, nonfinite values, 32-bit ABI, arbitrary random inputs or
concurrency, and do not resolve the preserved field-relation assertion.

Hyper already provides checked word/six-limb exact dyadic accumulation with
arbitrary-precision fallback. Its three existing focused regressions pass in
debug and release. No new production/donor change or performance claim is
retained; all 955 live source/support hashes still match checkpoint 27.

`nfloat-support-experiment.json` binds 39 evidence/support files, nine terminal
gates (including preserved initial driver compilation and sandbox inventory
failures), seven read records, source maps, executable and linked libraries.
Run `node verify-nfloat-support.mjs --derivative-live` for all 28 chained
verifiers. Recorded run passes: 28 JSON lines/39,741 bytes, empty stderr, exit 0.
Both pinned inventories and deduplicated coverage also verify. Only one
27,928-byte dedicated executable was added in /tmp; existing library/build
caches were reused and about 23 GB remains available, not a quota guarantee.
No deletion, donor patch, commit or push. All earlier evidence stays preserved.

## Checkpoint 27: derivative qualification and retention

Retained the two demand-bounded polynomial Horner loops in live Hypercurve,
with the identical 170-line regression module from the qualified candidate.
All 955 live source/support hashes match that frozen candidate; the scalar,
solver and other consumer sources remain unchanged. This is a workload-specific
performance/allocation transfer, not a new exactness claim or ecosystem closure.

The full all-feature candidate library/integration suite passes 1,764 tests,
with the same nine ignored tests and exact previous membership plus three new
regressions. Live debug/release focused tests and formatting pass. All-feature
Clippy passes. Each variant passes 1,584 exact state queries in debug, release
and Memcheck, plus ten domain/capacity rejections. The probe covers serialization,
progressive warming, cancellation recovery and four-worker shared/fresh inputs.
Cancelled observations are discarded; it does not prove that cancellation was
observed or qualify arbitrary concurrent state histories.

The earlier 80-group generic-parameter benchmark still supplies the large
high-order benefit and its one slower low-order control. New endpoint-consumer
measurements add 768 CPU observations, 12 ABBA blocks over 16 groups, ratios
0.912326–1.012088: five per-group intervals below one, none above. These known
closed-square traversals call the public tangent-order API; the affected end
requests only order one. Allocation/peak/live measurements match in all 16
groups over 96 observations; one group retains equal additional bytes after
eight queries. No universal speedup or steady-state endpoint-memory claim.

A separate 320-observation generic-query staircase checks 1/8/32/128/512 measured
queries after the original eight warm queries. All 16 selected groups have
identical baseline/candidate peak/live deltas, plateauing from 32 through 512;
maximum shared live delta is 196,376 bytes. Incremental request/byte rates also
match between the last two intervals per variant. This is empirical bounded
behavior for these cases, not an arbitrary-program memory bound or RSS result.

Endpoint Memchecks report zero errors/lost blocks and 63,176 reachable bytes
each. Both threaded state Memchecks return 97 for the same 48-byte possible
loss in Rust thread initialization seen in the prior independent thread control,
with zero definite/indirect losses and 347,608 reachable bytes. They are not
clean memory gates. All-feature WASM fails identically in both variants through
comparative-benchmarks -> curvo -> rand -> getrandom 0.4.2. Explicit ordinary
library features compile on WASM; no WASM execution or all-feature pass is claimed.
Initial endpoint harness API compilation failures and original source are kept.

Matched stripped basic/arrangement examples grow 224/208 bytes and pass their
assertions. The high-order work savings, modest endpoint results and preserved
exact outputs justify the tiny algorithm/source and representative binary costs.
The production algorithm adds six/removes four lines; test registration adds
four lines besides the new module. No donor edit, deletion, commit or push.

The nfloat support pass reads four whole files and scalar lines 1..310, adding
1,653 lines. Cumulative continuation coverage is 781 complete/13 partial files,
84,671 lines. Fixed-limb precision and precision-dependent matrix dispatch are
ideas to compare locally, not qualified replacement backends. Experimental
directed rounding, test limitations, boundary handling and the earlier
field-relation assertion remain open along with the full original inventory.

derivative-qualification-experiment.json binds 150 source/support/evidence files,
40 terminal gates (including six expected failures), five read records, both
955-file snapshots and 14 binaries/148,651,888 bytes. Run
`node verify-derivative-qualification.mjs --derivative-live` for all 27 chained
verifiers. Recorded verification passes with 27 JSON lines/37,650 bytes, empty
stderr and exit 0. Both full pinned inventories and deduplicated coverage verify.
About 148.7 MB of dedicated executable snapshots were added; the shared Rust
target is about 13 GB and /tmp has about 23 GB available, not a quota guarantee.
Prior snapshots, failed evidence and the 78 MB crash dump are preserved.

Next: remaining nfloat scalar/conversion/dot/matrix kernels and field-relation
support, unresolved earlier transfer ideas, and every other original reference.
Rank v1 remains isolated/unselected. Counts and decisions below are historical.

## Historical checkpoint 26: derivative-demand costs and LLL support

The derivative candidate remains isolated and unretained. Its only production
difference is two active-order bounds in Hypercurve's polynomial derivative
Horner loops; requested output lengths and rational quotient orders are unchanged.
Identical 170-line regression modules pass three tests in both variants and
debug/release. Source validation pins 955 files per variant, allowing only the
specified candidate edits. All 954 retained live files remain unchanged.

The public-API campaign has 3,840 CPU and 480 separate allocation observations
over 80 groups: two curve families, four degrees, five derivative orders and
retained/fresh-curve lifecycles. Every process prechecks exact results then warms
eight queries. Fresh construction uses prebuilt shared scalar controls; parameter
is 1/3. Timings use 12 alternating ABBA blocks pinned to CPU 6; allocations use
three pairs. Recorded builds and memory checks do not overlap CPU measurement.
There is no endpoint-helper CPU or cold-scalar/steady-state claim.

Paired ratios range 0.146542–1.058681; 55 per-group intervals lie below one and
one lies above one (degree-one/order-zero pi-scaled retained control, ratio
1.042047). The degree-eight/order-128 retained polynomial falls from 155.317 to
22.623 microseconds/query, and from 1,733 requests/223,984 bytes to five requests/
44,272 bytes. Requests/bytes fall in 16 groups, equal in 64, increase in none.
Peak/live deltas are identical in all groups. Fourteen groups retain additional
bytes during measurement in both variants, up to 126,728 bytes; this is not a
steady-state or leak diagnosis. Bootstrap intervals are not multiplicity-adjusted
or universal. Raw observations and independently recomputed summaries are bound.

Matching high-order public Memchecks have zero errors/lost blocks and 264,184
reachable bytes each. Candidate formatting passes. Initial test-only constructor
compilation failure and source are retained alongside corrected successful runs.
Linked probe files shrink 112/88 bytes, but text falls 316 and BSS rises 320
bytes each; representative stripped-example qualification remains open.
Next: full feature/consumer/state, endpoint workload, Clippy/WASM-compile and
representative-size gates before a production retention decision.

The source pass completes is_reduced_d.c plus 14 support files, 2,980 new lines.
Current is_reduced_mpfr uses nfloat contexts. Float imports check DBL_MAX range,
not exact representability; large integer exports truncate toward zero regardless
of hardware rounding. Generic/exact validation distinguishes operation failure
from comparison results. Full nfloat/field-relation support remains open.
All 3,360 closed-form LLL controls pass across dimensions 0..6, four patterns,
five power-of-two scales through 2^1200, basis/Gram inputs, four caller rounding
modes and three routes. Binary64 leaves 80 reduced cases uncertified; later
routes decide them. All calls preserve rounding mode. Native/Memcheck rows match,
zero errors/all allocations freed: 179,867 requests and 42,930,504 total bytes.
This does not explain the prior field-relation assertion or replace its failed
memory evidence. No donor assertion or relation check was changed.

Cumulative coverage is 777 complete/12 partial files, 83,018 lines; this is only
the Calcium/FLINT continuation, not ecosystem closure. derivative-demand-experiment.json
binds 70 source/support/evidence files, 15 terminal gates, 15 read records, both
955-file snapshots and five binaries/14,802,448 bytes. Run
`node verify-derivative-demand.mjs` for the 26 historical chained verifiers;
the former --monic-live check describes the pre-derivative live snapshot.
Both pinned inventories and deduplicated coverage verify. Previous counts below
remain historical. No new production/donor edit, deletion, commit or push.
The shared Rust target was reused (about 11 GB); dedicated binaries occupy
14.8 MB in /tmp/calcium-derivative-costs.FA5c2M, while source copies are in the
workspace. About 25 GB /tmp availability remains, not a quota guarantee.

## Previous checkpoint: spectral source closure and matrix functions

Both ca_mat source directories are now completely read at the pinned versions:
107 archived Calcium files/8,834 lines and 109 current FLINT files/7,486 lines,
including their tests. This pass adds 63 complete files and 4,935 read lines.
Called generic matrix, algebraic and LLL support and the full original ecosystem
remain open. Cumulative coverage is 762 complete files, 13 partial files, 80,038
lines. coverage.json preserves historical records; coverage-extensions.json adds
new ranges to checkpoint-bound partial records without editing those old bytes.
effective-coverage.mjs unions ranges for totals, so overlapping reads are not
double-counted. Earlier checkpoint counts below remain historical.

Jordan block construction uses eigenvalue multiplicities and rank differences of
powers, followed by generalized-kernel chains. Current generic matrix functions
reuse one maximal normalized jet per eigenvalue, copying prefixes to its blocks;
log inverse powers use balanced multiplication before order scaling. This suggests
reuse and demand-bounded coefficient work, not a direct ownership or matrix API
port to Hyper. Hyper already has exact factorial/basis recurrences, bounded
coefficient construction and fixed-size structured matrix packages. Its generic
polynomial derivative loops still merit a separately qualified active-order
bound; no candidate or performance claim is made here.

The known-block Jordan-form test constructs a dense similar B but calls the
routine on original A. Other assertions often condition on success and accept
Unknown equality. The Jordan-block rational family does require success, and the
DFT tests require True for square inverse products through dimension 16; neither
fact establishes the missing dense Jordan-form or general-function coverage.

All 486 new Jordan rows pass across three scalar shifts (2/3, sqrt(2), log(2)),
nine block patterns through dimension six and three similarity shapes. Six routes
cover blocks, full separate outputs, J/input alias, P/input alias, J-only/null P,
and supplied decomposition. Exact block multisets are required; the 324 routes
requesting P also require AP=PJ via explicit scalar sums and nonzero determinant
via the independent rational-matrix subsystem. No CA matrix multiplication,
inverse, determinant or Jordan constructor supplies the input/chain oracle.
Scalar arithmetic and the rational backend remain shared; no full branch claim.
Native/Memcheck rows match, zero errors/all blocks freed (1,092,304 allocations/
frees, 54,643,520 cumulative bytes). Archived mechanisms were read, not executed.

The 648 native matrix exp/log rows add zero, one and negative-one shifts, finite
Jordan-block coefficient formulas and fresh/whole-alias outputs. All statuses
match known existence: 66 singular-log failures and 582 successful results.
Of those results 568 exact equalities are proved and 14 remain Unknown, all in
the three-distinct-eigenvalue pattern with quadratic/logarithmic shifts. No
incorrect value was observed. Finite coefficient recipes avoid tested matrix
function kernels but share scalar elementary functions and the standard exp
factorial recurrence; these are not separate arbitrary-real accuracy oracles.

The function Memcheck aborts with SIGABRT at an internal LLL reduced-basis
assertion during CA field-relation discovery. Its 4,096-byte stdout is only a
buffered prefix (168 complete rows plus a fragment), not evidence of the exact
failing case or completion. No invalid-access/uninitialized-use diagnostic is
reported, but 15 possible-loss contexts remain after termination: 136,296 bytes
in 4,065 blocks plus 120,712 reachable bytes. This is not a clean memory gate or
proof of normal-exit leaks. The final LLL validation tail was read; full cause,
environment sensitivity and native reproducibility remain open. No assertion,
relation check or diagnostic was disabled. The original log and 78,024,704-byte
crash dump are preserved; no minimized memory-failure probe was introduced.

spectral-experiment.json binds 28 source/support/evidence files, six terminal
gates (including the expected Unknown exit and aborted memory run), 66 coverage
entries, one extension, two binaries totaling 40,736 bytes and the crash dump.
Run `node verify-spectral.mjs --monic-live` for all 25 checkpoint verifiers and
954 unchanged retained live source/support hashes. Both full pinned inventories
verify; `node effective-coverage.mjs --effective-summary` includes the 110-line
documentation extension omitted from the historical inventory reporter's base
coverage totals. A passing evidence verifier preserves failures; it does not
convert them to successful tests. No production/donor change, deletion, commit
or push. Rank v1 remains isolated and unselected. New /tmp use is about 41 KB;
the crash dump was moved intact from the comparison root into results in the
workspace. Approximately 26 GB /tmp filesystem availability remains.

## Previous checkpoint: solve/characteristic certificates and scalar domains

No new production or donor change. This pass adds 32 complete independent source
reads plus supporting documentation ranges, 5,913 lines. Combined coverage is
699 complete files, 11 partial files, 75,103 read lines. Exact ranges and notes are
in charpoly-domain-read-selection.json and coverage.json. This is not full Calcium/
FLINT or ecosystem completion; previous checkpoint counts below are historical.

Read remaining triangular/nonsingular solve tests, Danilevsky characteristic
polynomials, companion matrices, eigenvalue/diagonalization wrappers and tests at
both pins, plus current generic Danilevsky, the full 1,831-line CA method wrapper,
its test registration and vector header. Partial generic implementation and
domain/status documentation reads are explicitly bounded. CA has no specialized
vector-dot override: its generic strided dot uses scalar operations, not an
optimized joint CA dot. Hyper already has pivot-free determinant construction and
algebraic-fiber Berkowitz support. No missing capability or worthwhile new
workspace/representation transfer is demonstrated.

All 4,464 solve controls pass: 3,024 triangular rows directly exercising default,
classical and recursive routes, plus 1,440 nonsingular-API rows over four methods
(768 nonsingular successes and 672 certified singular failures). Empty dimensions,
multiple RHS counts, rational/sqrt(2)/log(2) scalars, initialized separate outputs,
whole RHS aliases and ignored stored unit diagonals are sampled. Failed outputs
are not read. RHS recipes use integer coefficients and scalar arithmetic, not
the tested matrix multiply. No invalid-pointer, failed-rank-state or blanket
overlapping-storage alias test is involved.

All 432 characteristic-coefficient controls pass for default and Danilevsky,
dimensions 0,1,2,3,4,5,6,8,10 and the same three scalar fields. Known diagonal/
triangular spectra, explicit dense similarity and reversed basis order are checked
against the integer recurrence for the product of (t-k), scaled by the common
scalar. Output polynomials are either fresh or seeded to length 17. This avoids
charpoly/determinant/matrix-polynomial oracles, but shares scalar arithmetic and
does not assert all branch coverage. Current native/Memcheck rows are identical,
zero errors/all blocks freed. Archived counterparts were read, not executed.

The generic CA wrapper has domain/property failures. Among 580 valid-input rows,
12 RR asin/acos outputs for ±2 report GR_SUCCESS despite being non-real; the two
algebraic contexts incorrectly report the real-vector-space property. Another
18 successful algebraic-context Arg outputs have Unknown algebraicity predicates.
A separate exact witness proves all are −pi, hence transcendental. Across all four
contexts, 36 negative-rational rows reject the correct +pi principal oracle and
match −pi. This reproduces checkpoint six's phase defect; it is not 36 independent
new bugs, and the negative-pi diagnostic does not change the principal oracle.
Zero/seven initialized separate outputs and whole aliases repeat each input.
The initial compile failure (nonexistent ca_ctx_ptr typedef) and its exact source
are preserved separately from the corrected successful build.

Domain and +pi witness processes exit one for mathematical failures, while their
Memchecks report zero errors/all allocations freed. The negative-pi value witness
exits zero; it diagnoses the wrong value, not correct principal behavior. Hyper's
existing public asin/acos guards pass 102 rows per debug/release profile, covering
rational boundaries and near-boundary values, radicals and ±pi, clone/serde/
explicitly warmed inputs. Successful results refine to −80 bits; invalid inputs
return NotANumber. This is domain/refinability qualification, not independent
transcendental numerical accuracy or guaranteed cold construction. Release
Memcheck matches the rows, zero errors/no lost blocks, 3,112 reachable bytes.

charpoly-domain-experiment.json binds 72 source/support/evidence files, 19 terminal
gates (including expected failures), 36 read entries and seven binaries totaling
6,203,896 bytes. Run `node verify-charpoly-domain.mjs --monic-live` for all 24
checkpoints and the 954 unchanged retained live source/support hashes. Full capture
has 24 JSON records, terminal zero and empty stderr; both full pinned inventories
also verify. No new CPU/allocation/binary-delta claim: no production candidate was
introduced. Rank v1 remains immutable and unselected as implemented. Reused the
9.8 GB shared build target and kept the 6.2 MB binary evidence in
/tmp/calcium-charpoly-solves.lbZlN8; about 26 GB filesystem availability remains.
No historical evidence, donor or live production edit, deletion, commit or push.

## Previous checkpoint: rank costs and matrix support

The isolated rank v1 is **not selected as implemented**. Its 204 additional exact
decisions from checkpoint 22 remain verified, but unresolved same-order searches
are too costly in the sampled corpus. The mathematical witness idea stays open
for better scheduling/reuse; no production change, arbitrary search cap or
redefinition of the required completeness work is made.

Matched CPU has 2,688 observations:56 groups ×12 alternating ABBA blocks ×4 runs,
on CPU 6 with uninstrumented binaries. Separate allocation has 336 observations,
three baseline/candidate pairs per group. Seven patterns/four widths/two lifecycles
distinguish known controls, newly certified witnesses and still-unresolved inputs.
Both lifecycles use a shared prechecked tiny-positive scalar and eight warmups;
fresh_problem rebuilds/analyzes the Problem but is not cold scalar construction.

At width 32 with retained analysis, unresolved one-row and two-row cases have
paired median ratios 7.975 and 48.007; the latter per-group bootstrap interval is
42.763–50.020. Its requested bytes/query increase 47,320→1,141,240 (24.117×), with
requests 527→5,597. Newly certified one-/two-row witnesses cost 14.771×/31.050×
baseline Unknown, which is different work, not equal-work speedup evidence.
Known-zero controls also slow (paired ratios 1.078–1.218); other known controls
span 0.988–1.118. All estimates are scoped to this host/corpus; bootstrap intervals
are per-group, not multiplicity-adjusted or universal bounds.

Requested allocations/bytes increase in 32 groups, equal in 24, decrease in none.
Incremental requested peaks increase in 24, equal in 32; measured live deltas are
zero. These counters exclude native/allocator overhead, RSS and stacks. CPU/
allocation driver files change−312/−296 bytes, with text−416 and bss+416 each;
these include build-path/layout, are not application sizes, and do not establish
a code-size win. No broader state, serde, concurrency, downstream, Clippy or WASM
qualification is claimed for this unselected version.

Completed 40 more independent full-file reads,2,521 lines. Matrix multiplication
routes rational operands through borrowed headers and common fields through
shared-denominator polynomial products; those ownership layouts are not a Rust
port. Matrix polynomial evaluation uses Paterson–Stockmeyer blocks and contains
an unused allocated matrix temporary. Hyper already shares adjugate/determinant
work for fixed 3/4 division with exact-rational gating, and uses specialized
repeated squaring. Workspace reuse/high-degree evaluation remain ideas to compare,
not demonstrated missing Hyper capabilities or performance gains. Matrix
predicates already let a later False dominate an earlier Unknown, which motivates
but does not cost-justify combinatorial rank search.

New native checks require True in all 1,528 rows:408 multiplication controls and
560 each for powers and polynomial evaluation. Dimensions 0–8, rectangular/empty
products, separate/left/right/both whole aliases, seeded outputs, rational,
quadratic/logarithmic scalars and large denominator inputs are covered. Powers/
polynomial lengths sample 0,1,2,3,4,7,8,15,16,31. Integer dot coefficients and
Jordan-block binomial coefficients avoid tested matrix kernels; scalar backend
is shared. Aliased classical multiplication may delegate to default dispatch,
and simultaneous input alias uses identical operands; not full branch coverage.
Native/Memcheck rows agree, both exit 0, zero memory errors/all allocations freed.
Archived code was read independently but not executed.

Combined source coverage is 667 complete/eight partial files,69,190 lines. Exact
ranges/notes are in rank-cost-read-selection.json and coverage.json. Full original
ecosystem inventory, remaining matrix/algebraic/generic support and unselected
transfer ideas remain open. Earlier checkpoint statuses below are historical.

rank-cost-experiment.json binds 41 source/support/evidence files,seven terminal
gates,40 read entries and five binaries totaling 10,956,864 bytes. Run
`node verify-rank-costs.mjs --monic-live` for all 23 checkpoints and 954 unchanged
live files. The checker recomputes every ABBA group, adaptive iteration count,
bootstrap interval, allocation range and native row; full output capture and both
pinned inventories pass. Shared target remains 9.8 GB; about 11 MB new binary evidence
is preserved in /tmp/calcium-rank-costs.lbsszv. No deletion, commit or push.

## Previous checkpoint: matrix solve outputs and isolated rank dominance

Added50 complete files and172 documentation lines,4,229 lines altogether. Both
pins' inverse/solve/rank/RREF/kernel/adjugate paths and seven relevant tests were
independently read; current generic triangular substitution was also read fully.
Combined coverage is627 complete/eight partial files,66,669 lines. Supporting
matrix polynomial evaluation, other kernels/tests and the full ecosystem remain
open. Exact reads/notes are in matrix-solve-read-selection.json and coverage.json.

Current generic substitution computes diagonal inverses once across RHS columns,
with direct-division fallback for nonfield contexts; archived code divides per
column. Shared adjugate construction may reduce repeated Cramer work for multiple
RHS. Hyper already shares augmented fraction-free elimination and residual proof
replay; inverse-node reuse, symbolic forms and workload costs require qualification.
Retaining pivot metadata can avoid rediscovery of established facts; no donor
mutable ownership or failure-status pattern is selected for direct transfer.

New native probe has1,074 valid-input output rows. Default/LU zero-RREF returns
success/rank0 but preserves seeded nonzero output in144 rows. FFLU, fresh-zero
outputs and whole-input controls pass. Order2 in-place cofactor/default adjugate
adds six incorrect rows and inverse three; charpoly in-place controls pass.
Four larger quadratic cofactor rows are Unknown, consistent with the earlier
minor-LU dispatch gap, not wrong values. Thus157 required-True checks fail:
153 incorrect/four unresolved. These repeat dimensions/fields/routes, not157
distinct defects. The matrix oracle uses closed-form upper-bidiagonal integer
coefficients, not determinant/adjugate/inverse kernels; scalar backend is shared.

Native and Memcheck output agree, both exit1 for mathematical failures, while
Memcheck reports zero errors/all allocations freed. No invalid-access or failed-
rank-state probe. Archived mechanisms were read, not executed. Documentation does
not promise blanket alias support; whole-input discrepancies are reported with
that limit. Read donor tests use separate inverse/adjugate outputs and zero or
input-copy RREF outputs, accepting Unknown or conditioning checks on success.
The earlier28-function donor gate cannot rule out these independent controls.

An isolated774-file rank candidate changes only hypersolve/src/rank.rs: continue
past Unknown at the same minor order, accept a later nonzero witness, but never
descend while that order remains unresolved. In828 public queries per variant/
profile, baseline606 certified/222 Undecided becomes810/18, with correct rank/DOF
and18 preserved unresolved controls. The204 gains repeat patterns across precision
floors and witness positions. Scalar states are prechecked/shared within groups;
these are not cold-start or arbitrary-history claims.

Both profiles and focused Memchecks agree exactly per variant. Hyper Memchecks
have zero errors/no definite,indirect or possible losses, with2,272 reachable
bytes. Existing803 solver tests pass per profile with unchanged membership;
formatting passes. **Not retained:** scanning unresolved minors may be expensive.
Matched CPU/allocation/size, broader state/serde/concurrency, focused in-crate
regressions, downstream, Clippy and WASM qualification remain open.

matrix-solve-experiment.json binds50 files,12 terminal gates,51 read entries,774
candidate source hashes and five preserved binaries (14,316,952 bytes). Run
`node verify-matrix-solves.mjs --monic-live` for all22 checkpoints and unchanged954
live files. Full-output capture matrix-solve-verify-full has22 JSON records and
terminal0; both pinned inventories verify. Shared target is9.8GB, with about14.3MB
new binary evidence. No production/donor edit, deletion, commit or push.

## Previous checkpoint: matrix capability audit and retained monic normalization

The continuation now has577 complete files,seven partial files and62,440 read
lines:47 additional matrix/support files and4,517 lines at the two pins. The
complete original ecosystem inventory and remaining matrix/algebraic/generic
support are still open. Per-file ranges and findings are in coverage.json; the
new read selection is matrix-pivot-read-selection.json.

Current stride-based matrix storage, cheapest-certifiable pivots, recursive LU,
fraction-free updates, determinant dispatch, Berkowitz and fixed-size cofactors
provide useful comparisons. Hyper already has contiguous fixed matrices, cheap
Vec row swaps, exact integer cross-differences and pivot-free Faddeev–LeVerrier
determinants. No unconditional storage/algorithm replacement is selected.

Native198 structured cases show33 LU and ten default-dispatch Unknown results
for certifiably singular matrices; Berkowitz/Bareiss prove all expected results.
All28 donor matrix test functions pass, but the read determinant tests accept
Unknown and avoid large default dispatch; the read LU tests use rank_check=0.
Registry/execution coverage does not credit unread constituent test files.
Hyper constructs all198 determinants and proves182 equalities, leaving16
logarithmic comparisons unresolved. Every sampled singular case is proved zero.
Native and Hyper focused Memchecks are clean; no incorrect answer is observed.

A separate isolated method comparison adds112 dense inputs with an independent
integer permutation determinant oracle. All310 inputs/930 queries per profile
match across debug/release/clean Memcheck. Bareiss/Faddeev each prove260 and leave50
Unknown; Berkowitz proves266/leaves44, gaining eight but losing two comparisons.
**No matrix production transfer.** Alternate scheduling remains open; these are
capability results, not matched timings, allocation measurements or universal
algorithm dominance. Donor polynomial/scalar references still share their backend.

**Monic normalization is now retained in live Hypersolve.** Full frozen downstream
Hypercurve finishes with1,761 passed/nine pre-existing ignored across45 suites,
exactly matching baseline membership. All954 live source/support files match the
qualified snapshot. Live debug/release each pass803 tests, including three new
regressions; Clippy, fmt, metadata and WASM compile-only gates pass. Relative to
the previous live snapshot, only root_isolation.rs and its new101-line test file
change. No public API, dependency, scalar representation or cache-field addition.

The three coefficient/two root proof improvements and lower allocation demand
justify this narrow retention under completeness-first priorities. Accepted costs
remain explicit: timing ratios0.9007–1.0667 include slower controls; two stripped
examples grow2,944 bytes each including layout effects. Prior state Memcheck is
still non-clean: baseline, candidate and std-only thread control all report the
same unsuppressed48-byte possible loss. The new matrix gates do not erase it.

matrix-pivot-experiment.json binds47 files,11 gates,47 reads and six binaries
(12,399,752 bytes). retained-monic.json binds25 files,six live gates and954 source
hashes. Run `node verify-retained-monic.mjs --monic-live` here for all21 checkpoints
and current production bytes. Full-output capture retained-monic-verify-full has
21 JSON records and terminal0; an earlier empty nested capture is preserved and
not credited. Both pinned inventories verify. Reused shared builds; no donor edit,
deletion, commit or push. Historical sections below remain as-of their checkpoints;
their optional --retained-live flags intentionally reference the pre-monic snapshot.

## Previous checkpoint: monic state/size and complete polynomial directories

Eight more files/579 lines complete both ca_poly directories, including tests:
74 archived files/6,371 lines and70 current files/5,571 lines. Combined coverage
is530 complete/seven partial files,57,923 lines. Further generic support, matrices,
algebraic paths and the original full ecosystem inventory remain open. The archived
atan-series test has no mathematical assertion; composition/full-power tests
accept Unknown. All15 current donor polynomial tests pass, without strengthening
those oracles. A new scalar-recurrence full-power probe passes540 cases/1,080
required-True checks, including in-place outputs, with clean native Memcheck.

The new774-file consumer snapshot changes the already measured monic candidate
only by adding three cfg(test) regressions; runtime body is unchanged. All803
Hypersolve tests pass per profile, with old800 membership plus three focused tests.
Clippy, fmt and WASM library compilation pass. WASM execution/full CI are not claimed.

Both variants pass1,932 public queries per profile:81 recipes ×22 lifecycle states
plus150 expanded degree/scale inputs. Serialization, three warming floors,
cancellation recovery and four shared/independent workers preserve1,646 known/286
Unknown outcomes; all4,836 projective and4,836 output-roundtrip coefficient checks
per variant prove equality. All81 input serializations are stable. Candidate clears
154 coefficient/44 residual Unknown comparisons across repeated states, retaining
132 unresolved residuals. These repeat three coefficient/two root improvements,
not new independent identities. Expanded multiplicities reach total degree24 and
rational scales2^-300 through2^300. No valid observation is used while aborted.

State Memcheck is **not a clean gate**: both variants and a separate std-only thread
control report48 possible-loss bytes at Rust thread initialization and exit97.
No definite/indirect loss or invalid/uninitialized accesses are reported. Sequential
--vgdb=no repeats remove initial pipe-unlink noise and preserve every output row
and the48-byte result; no suppression. Debug/release/all memory-run rows agree.

Two stripped Hypercurve examples each grow2,944 file bytes and pass their assertions.
Text/data change+3,008/−64; bss/layout makes loaded totals change0/+4,096 bytes.
These include build-path/layout effects, not pure algorithm-code attribution or
whole-application size. Prior7,776 CPU/972 allocation observations still apply to
the unchanged implementation, including selected slower timing controls.

**Not retained yet:** full Hypercurve downstream qualification is running and is
not bound here. monic-state-experiment.json binds102 source/support/evidence files,
28 terminal gates (five expected memory exit97 outcomes),774 candidate source hashes,
11 binaries totaling71,443,536 bytes and8 read entries. Run
`node verify-monic-state.mjs --retained-live` here to verify all19 checkpoints and
the953 retained live files. No production/donor edit, deletion, commit or push.

## Previous checkpoint: monic costs and polynomial implementation closure

Added46 complete independent reads/2,740 lines. All56 archived and53 current
top-level ca_poly C files are now read, including composition, storage and
conversion; seven generic support kernels were also read. Combined coverage is
522 complete/seven partial files,57,344 lines. Polynomial tests, further support,
whole donor libraries and the full ecosystem remain incomplete. The newly read
binary-power callee corrects earlier wording: binexp is not binomial expansion.

The unchanged isolated monic candidate has7,776 uninstrumented CPU observations
over81 recipes/two lifecycles/12 alternating ABBA blocks, with no recorded build
or checker overlap. All162 group medians and paired bootstrap intervals reverify.
Trial/baseline ratios range0.9007–1.0667; selected controls cost2–7% more. Two
retained logarithmic cases improve by about8.5% and9.2% paired, while several
other intervals include parity. This is not a universal speedup or a timing of
the newly available downstream proofs. Same Some/None and degree outcomes do
not imply identical symbolic output. Eight warmup queries exclude cold-start claims.

Separate972 allocation observations show lower request counts/bytes in56 groups,
lower incremental requested peaks in44 and no increases in this corpus. Live
deltas match baseline; nine-recipe108-observation churn controls show no growth
from1 to1,000 queries. Requested Rust bytes are not peak RSS or total native heap.
The unstripped CPU driver grows1,400 file bytes/text36/loaded total4, not an
application-size conclusion. Candidate strict Clippy and WASM library compile
pass; prior800-test debug/release suites remain bound to the identical source.

Native composition/storage controls pass384 cases/4,608 required-True checks,
including whole-input aliases, sparse/dense inputs, padded reversal/shifts and
context transfer. Scalar power/convolution references share the donor scalar
backend. Memcheck is clean with every allocation freed. A separate existing
Hyper fraction-free-chain probe gives128 known/34 Unknown across162 queries in
both profiles, with correct known degrees and clean Memcheck. It resolves none
of the13 original logarithmic gaps and loses four additional family cases, so
no direct replacement/fallback is selected for this corpus.

**No production transfer.** Focused in-crate regressions, broader degree/scale
and lifecycle/serde/concurrency, downstream and representative application-size
qualification remain pending. The complete original reference inventory remains
in scope. Source notes and all raw successes, costs and unresolved outcomes are
preserved; no unrelated source edit or deletion.

monic-cost-experiment.json binds65 files,13 terminal gates,seven binaries
(15,597,136 bytes) and46 read entries. Run
`node verify-monic-costs.mjs --retained-live` here to verify all18 checkpoints,
every raw cost row/interval and the953-file retained production snapshot.

## Previous checkpoint: polynomial roots and series (candidate not retained)

Added34 complete independent reads/4,493 lines across the two pins. Cumulative
coverage is476 complete/seven partial files,54,604 lines; full Calcium/FLINT and
the ecosystem inventory remain incomplete. Per-file notes and ranges live in
coverage.json and polynomial-closure-read-selection.json.

Native dense series controls pass280 cases/5,880 required-True checks; sparse
monomials pass756 cases/3,024 checks. Independent scalar recurrences cover inv,
div, log, exp and truncated powers, whole-input aliases, short/zero orders and
rational/shared/mixed fields. This is not independent scalar-backend validation
or instrumented branch coverage. The81-case root corpus reconstructs squarefree
factors and matches roots one-to-one with exact multiplicities; all cases pass.
All three focused native Memcheck runs have zero errors/all allocations freed.

An initial Hyper harness failed because it demanded a particular output scale,
which square_free_part does not promise. Its source and failed gate remain
preserved. A separate projective diagnostic observes68 correct-degree results
and13 unresolved cases. All returned polynomials are provably proportional to
their known-factor reference. Three direct coefficient cases and selected root
residuals remain undecided; none is disproved.

The isolated339-file monic candidate publishes the proved leading coefficient
as exact one instead of rebuilding a*(1/a). This clears all three coefficient
cases and two root queries, without changing the68/13 result split. Baseline
and candidate outputs each match across debug/release/Memcheck; focused Memcheck
has zero errors/no definite, indirect or possible loss, but retains reachable
state. Candidate800-test solver suites pass in both profiles with unchanged test
membership, and fmt passes. Six root-residual queries remain unresolved.

**Not retained.** Matched CPU/allocation/size, state/serde/concurrency, downstream,
Clippy/WASM qualification and focused in-crate regressions remain pending. The
prior fact-first solver's measurements do not qualify this candidate. Live
production remains the previously retained953-file scalar/consumer snapshot.

polynomial-closure-experiment.json binds74 files,19 terminal gates, eight binary
snapshots (17,095,376 bytes), the native library hash,339 candidate source hashes
and34 read entries. Run `node verify-polynomial-closure.mjs --retained-live` here
to verify all17 chained checkpoints and current production bytes. A nested draft
capture emitted no stdout and is not evidence; direct and approved full-output
verification are recorded separately. No new production/donor edit or deletion.

## Previous checkpoint: retained fact-first polynomial decisions

A separate v2 candidate preserves v1's proof domain while consulting existing
nonzero facts from the leading end before refining unresolved lower terms. It
does not treat Unknown as zero or guess mathematical degree. It adds no scalar
state or public representation. All630 public rows match v1, with120 more
decisions than baseline and60 preserved unresolved-leading controls.

The new three-way CPU campaign has2,592 usable observations,36 groups and12
rotating forward/reverse blocks onCPU6, without build/checker overlap. Degree16
retained log-self falls from67.286us in v1 to4.549us in v2; tiny-self falls from
42.688us to4.323us. Same-outcome v2/base control ratios span0.973–1.029. The
baseline returns Unknown for the newly decided cases; those are different-work
comparisons, not speedups over an equally complete baseline. The structural
fact prepass is not claimed to be constant-time for opaque scalar graphs.

All800 solver tests pass in each profile, as do1,090 state queries and64 byte-
stable serialization checks per profile. State controls include cancellation,
later refinement, strict-only leading signs, four shared workers and independent
deserialization. Clippy, fmt, public Memcheck and WASM compile-only gates pass.
Downstream Hypercurve passes1,761 tests across45 suites; nine pre-existing
ignored tests remain unrun. Exact membership matches the retained-scalar baseline.

Allocation qualification deliberately preserves a failed450-row run whose
live-byte determinism assumption was wrong. Request totals agreed, but existing
pointer-indexed weak-cache occupancy varied by144 bytes. A separate648-row
bounded campaign and27 longer churn controls pass; live tiny-graph retention
stays within the existing2,304-byte native bound. Retained operands add no live
requested bytes. Degree16 log-self requests fall from1,117 /160,976 bytes to
13 /7,760 bytes versus v1. Instrumented clocks are not CPU evidence; requested
live-byte deltas are not peak RSS or allocator overhead. The two stripped
Hypercurve examples grow576 and560 bytes, with passing runtime assertions.

This source pass adds32 complete donor/support files and2,959 lines, giving
442 complete/seven partial entries and50,111 read lines across the two pins.
Common-field packing and denominator hoisting are interesting adjacent ideas;
Hyper already has rational primitive-integer specialization, sparse collection,
shared-scale carriers and product-sum APIs. General packed field convolution
needs separate representation and workload justification. No such port is kept.
The180-case native polynomial control requires True on4,860 checks and passes
clean Memcheck with every allocation freed. It is scalar-convolution cross-
checking, not an independent arbitrary-real oracle or branch coverage proof.

**Retained in live Hypersolve.** Only `src/resultant.rs` changes:103 added/two
removed lines including three tests. No scalar representation, dependency or
cache is added. Live800-test debug/release suites, all-feature/all-target Clippy,
fmt and WASM library compilation pass. All953 scalar/consumer source/support
files match the qualified snapshot; the other772 consumer files and180 retained
Hyperreal files are unchanged. No commit, push, donor edit or deletion.

`polynomial-facts-experiment.json` binds118 files,29 terminal gates,17 binaries
and32 donor read entries. `retained-polynomial-facts.json` binds the transfer and
six live gates. Run `node verify-retained-polynomial-facts.mjs --retained-live`
from this directory to verify all16 chained checkpoints and current live bytes.
Without that option, historical verification remains valid after future changes.
The full ecosystem inventory remains incomplete. Earlier checkpoint prose below
is historical; the root continuation ledger records subsequent dispositions.

## Previous checkpoint: polynomial degree and three-valued zero decisions

Read40 additional complete polynomial headers, documentation, implementation and
test files at both pins (4,159 lines). Combined coverage is410 complete/seven
partial files and47,152 lines. New paths cover storage/normalization, proper
degree, comparisons, long division, GCD and their selected tests. Whole
Calcium/FLINT and the ecosystem inventory remain incomplete.

The donor distinguishes stored degree from certified mathematical degree:
normalization removes only proved zeros, while division/GCD require a nonzero
leading coefficient. Its zero predicate allows any proved nonzero coordinate
to override earlier Unknown coordinates. A current-native exhaustive81-case
four-coordinate corpus passes405 truth, length, properness and monic checks;
Memcheck has zero errors and all blocks freed. Donor GCD tests allow failed
algorithms and Unknown equality results, so they do not prove completeness.

This read exposed a concrete Hyper transfer: Hypersolve's subresultant helper
returned immediately on an unresolved lower coefficient, even when a later
coefficient was exactly one. The new630-case public corpus includes self-GCD,
constant-one perturbations and unresolved-leading controls, degrees1–4, every
lower position and three precision floors. Existing Hyper gives450 known/180
undecided outcomes. An isolated fallback that remembers the first unresolved
index but scans for a proved nonzero gives570 known/60 undecided in both profiles.
All120 gained decisions are correct and all60 unresolved-leading controls remain
unresolved. These are repeated settings, not120 independent identities.

Candidate source changes only `hypersolve/src/resultant.rs` in a339-file frozen
three-crate snapshot. Hyperreal is the previously retained exponential-proof
state. No live production file changes. Three new tests cover256 four-coordinate
truth combinations, public shared/coprime factors and uncertified leading degree.
Baseline797 and candidate800 tests across seven suites pass; candidate passes
debug and corrected release, Clippy all-features/all-targets with warnings denied
and formatting. Test names match the frozen baseline plus exactly the three new
tests. Candidate public Memcheck passes630 queries with zero errors/no definite,
indirect or possible losses;3,240 bytes remain reachable. An initially mislabeled
release command omitted --release and is preserved as a debug run, not credited
as release; the corrected release is independently captured.

CPU qualification records1,728 observations in36 groups: six coefficient/relation
cases, degrees1/8/16, fresh/retained operands and12 alternating ABBA blocks pinned
toCPU6. The first48 observations overlap formatting by65ms and are excluded.
Their286.879ms summed query clocks conservatively place every subsequent group
after formatting; all other builds/checkers are disjoint. Accepted scope is35
groups/1,680 observations. Same-outcome controls have paired ratios0.899–1.017
(17 groups), not a global performance guarantee. Newly decided degree16 retained
log-self cases cost67.849us versus2.918us baseline Unknown (paired23.072,
95%22.545–23.329); tiny-self43.967us versus2.287us Unknown (19.070,
95%18.774–20.128). Different outcomes are not equal-work speedups/regressions.
All lower coefficients in these benchmark cases share one scalar node; fresh
includes construction and teardown, retained operands have eight warm queries.
The unstripped linked driver grows72 file bytes; this is not application size.

The candidate remains isolated, pending a schedule that reuses already-certified
nonzero facts without shrinking the proof domain, allocation/state qualification,
broader consumer gates and size/retention judgment. Hyper's root-isolation code
already uses a certified-degree zero test and a retained divisor inverse, so the
same architecture is a relevant comparison. Calcium's numerical Sylvester
coprimality certificate is another open idea, not a transplanted approximation.
Archived-only integer-division/header and rational-GCD cleanup differences are
recorded as source observations; the archived backend was not executed here.

`polynomial-decision-experiment.json` binds74 files, five binary snapshots,18 gates
and40 source-read entries. `verify-polynomial-decision.mjs --live` passes all14
chained checkpoints, including exact public/test/native corpus membership, every
timing row/median/bootstrap interval, source hashes and explicit exclusions.
Both pinned inventories verify. All processes are terminal. New binary snapshots
use9.4MB under `/tmp/calcium-polynomial-decision.ItisYX`; shared builds use5.5GB
with about17GB quota headroom. Nothing was deleted, committed or pushed.

## Previous checkpoint: exponential relation proof retained in Hyperreal

The cache-preserving v3 proof is now retained in live Hyperreal. It proves exact
equalities and signs between supported positive exponential expressions by
canceling their logarithmic linear forms, without replacing numerical nodes or
serialized data. Exact opaque-atom comparisons use bounded weak-key reuse; a
cache miss still performs the original full comparison. Unknown never becomes
a guessed equality or sign. Two existing-file deltas and four new proof/cache/
test files were transferred byte-for-byte from the qualified snapshot. All
pre-existing user changes were preserved; no dependency, public type, version,
commit or push change was made.

Retention follows the requested exactness/completeness-first priority. The
earlier independently verified corpus adds152 exact downstream decisions out
of224 queries, and preserves all64 beyond-cap Unknown controls. It also preserves
the 48 root/exponential equality cases and96 newly decided tiny-sign queries
(repeated precisions/orientations, not96 unrelated formulas). Numerical, state,
serialization, concurrency, weak-lifetime and five-crate regression gates pass.
On live source,855 library/integration tests pass in each profile:780 library
and75 integration tests across14 suites. Exact names/statuses and Cargo target
membership verify; there are no ignored live Hyperreal tests in these runs.
All-feature/all-target Clippy with warnings denied, formatting and all-feature
WASM library compilation also pass. WASM was not runtime-tested; nine earlier
downstream ignored tests remain unrun. This is not a full-CI completion claim.

The final cost qualification adds four separately sequenced three-way campaigns:
5,184 numerical CPU/1,296 allocation observations across72 workloads, and1,728
first-touch CPU/432 allocation observations across24 sharing/depth cases.
All ordinary numerical allocation totals and all first-query Rust allocation
totals match v2. Numerical paired median CPU ratios span0.927–1.048 versus the
original baseline and0.907–1.032 versus v2. These observed small differences
are not universal speedups or an absence-of-regressions guarantee.

Fresh worker threads use parent-preconditioned numeric operands and constants;
query timers exclude thread start/exit. Independent depth128 unresolved first
queries take15.785 us versus v2 15.288 us and baseline4.186 us: paired ratios
1.030 (95%1.019–1.050) and3.772 (95%3.682–3.857). Sixteen subsequent queries
average2.590 us versus v2 9.055 us and baseline2.140 us: ratios0.284 (95%0.279–0.288)
and1.217 (95%1.173–1.229). Deep exact identities fall from v2 7.216 us to0.647 us
on warm repeats, with identical Equal outcomes; baseline returns Unknown.
Shared/common-tail inputs cost less than independent deep traversal. Across all
24 cases, first-query v3/v2 ratios range1.008–1.102. This is fresh TLS, not a
whole-process cold-start benchmark; concurrent throughput is not measured.

Measured memory/size costs are accepted explicitly: up to2,304 requested heap
bytes weakly retained per touched worker, released at thread exit, plus400 bytes
linked static TLS per thread on this host. The unmodified Hypercurve `basic`
and `arrangement` stripped release examples each grow8,656 bytes over baseline
(about0.08%); versus v2 they grow2,512 and2,464 bytes. All six stripped artifacts
execute their assertions successfully. These are representative examples, not
every application or an optimized-size/LTO profile. The source delta remains234
non-test-file lines/9,088 bytes, including test include directives. No global
bound on uncached paired-graph work or whole-process peak memory is claimed.

The donor audit also adds22 complete vector sources/headers/docs (1,595 lines),
closing those selected paths at both pins. Total Calcium/FLINT coverage is now
370 complete/seven partial files and42,993 lines, not full library completion.
Useful vector ideas are largely already present in Hyper: shared-scale facts,
fused known-exact products, componentwise negation, and a known nonzero coordinate
dominating Unknown coordinates in norm certificates. Current FLINT's public
`ca_vec_neg(v,v)` instead silently performs no work:24 wrong results among105
tests, with separate-output/raw in-place and zero/empty controls passing. All27
zero-truth combinations pass, and Memcheck reports zero errors/all blocks freed.
Archived code has the same mechanism but was not executed. Vector docs give no
per-function negation alias caveat; no guarantee of arbitrary overlapping
subranges or interior pointers across vector reallocation is inferred.

`reuse-cost-experiment.json` binds22 source/support and132 evidence files;
`retained-exp-proof.json` records the exact180-file retained state and18 live gate
artifacts. `verify-retained-exp-proof.mjs --live` passes all13 chained checkpoints,
recomputing every new benchmark row/median/interval, checking binary snapshots,
non-overlap schedules, donor controls, source ranges and live regression coverage.
Default verification preserves historical source validity independently of later
authorized production edits. All current qualification processes are terminal.
Build artifacts stay in `/tmp` (about4 GB shared target plus165 MB snapshots,
about19 GB quota headroom); no evidence or unrelated cache was removed.

This is one retained improvement, not ecosystem-audit completion. Separate
log/erf/eager-root prototypes remain unretained, and their open mathematical
ideas, all unread donor support, and the remaining full reference inventory
continue to be tracked in the root ledger.

## Previous checkpoint: v3 scalar/consumer and live-memory qualification

All scalar `ca/test` C sources have now been read independently, line by line:
29 archived files/3,400 lines and 30 current files/3,225 lines (including the
current test dispatcher). Together with current `ca.h`, this slice adds 52
complete files/6,132 lines. Combined coverage is 348 complete and seven partial
files, 41,398 reviewed lines. Both pinned inventories and every recorded range
verify. This is not full Calcium/FLINT or ecosystem completion.

The tests sharpen the donor comparison: many randomized algebraic identities
allow Unknown, and numerical overlap checks do not prove containment or decision
completeness. Strict special-value matrices are stronger but cover only listed
values. Random transfer inputs omit Arg; ordinary trig comparisons use separate
outputs; the initialization test does not exercise `randtest_same_nf`. These gaps
explain why the previously reproduced donor defects escape their local tests.
The properties suite explicitly expects Unknown for some mathematically settled
facts, including pi's irrationality and `exp(exp(-1000000)) != 1`; this is bounded
knowledge, not permission to return a false equality. No additional Hyper proof
rule is claimed from this read alone.

Frozen v3 independently passes 15,050 directed-MPFR enclosure checks over 43
inputs, 261 state/cancellation/serialization checks, 96 public equality cases,
and 144 tiny-delta sign cases in both debug and release. Query rows exactly match
the independently verified v2 corpus, preserving all its new decisions and
beyond-cap Unknown controls. A new stress harness interleaves eight workers'
proof queries, shared approximation refinement, and independently deserialized
operands: 1,732 sign queries and 3,456 directed enclosures pass in each profile.
An initial harness compilation failed on a private shift method; replacing that
call with public rational multiplication required no library change. The first
release job waited on Cargo's lock until after that correction and passed; its
later duplicate is retained as a separate run, not counted as extra coverage.

All 773 frozen consumer files match their baseline and corresponding live bytes;
that comparison does not inventory new live files. Hyperlimit (361 debug),
Hypersolve (797 debug), Hypertri (187 debug), and Hyperlattice (203 release) reruns
finish successfully. Hypercurve also finishes: 1,761 passed/nine pre-existing
ignored tests across45 suites. The original
Hyperlattice sandbox/compiler-cache failure and approved unchanged retry are
both preserved. The 224-query downstream corpus exactly preserves v2's 160 known
results/64 Unknown controls; focused Memcheck has zero errors and 7,208 reachable
bytes. All five crates total3,309 passed; exact test names, statuses, ignored
reasons and expected Cargo library/integration targets match the frozen baseline.
These are not full CI/both-profile consumer gates; overlapping clocks are not a
performance comparison and the nine ignored tests are not credited as passing.

A separate live-allocation campaign completes 162 observations: baseline/v2/v3,
1/8/32 fresh workers, 1/64/512 rounds, depths1/8, three repetitions. Workers pause
after dropping all expressions but before thread exit. Baseline and v2 retain no
requested heap at that boundary; v3 retains at most 2,304 bytes/32 dead node
allocations per worker in this corpus. The saturated 32-worker case retains
73,728 bytes/1,024 blocks. Every process returns to its pre-worker requested-heap
count after thread exit. Concurrent peak samples vary with scheduling and are
not a universal peak ratio or CPU benchmark. The runner's console capture is
empty despite success; complete raw162 rows and matching summary, not exit0
alone, establish coverage. These totals omit allocator overhead, RSS, stacks
and native TLS. Separately, linked ELF TLS grows848→1,248 bytes (+400) versus
baseline/v2, including threads that never use this proof cache, on this platform.

Eight-worker live-memory Memcheck is clean (1,344 reachable bytes). The new
concurrent proof-state Memcheck completes all numerical assertions but returns99:
one 48-byte possibly-lost `std::thread::current::init_current` allocation, zero
definite/indirect losses. A standalone standard-library-only scoped-thread
control reproduces the same 48-byte allocation/category/error count without
Hyper, GMP or MPFR. No suppression is installed and neither run is relabeled
clean; the record is not evidence of a v3-specific weak-key leak.

`reuse-qualification-experiment.json` binds30 source/support and104 evidence
files. `verify-reuse-qualification-checkpoint.mjs` passes all11 chained checkpoints:
source/evidence hashes, exact test and query membership, numerical check counts,
every memory row and summary, linked TLS sizes, scalar-test/header read closure,
and explicit failed-run/control dispositions. All qualification handles are now
terminal. The v3 candidate remains unchanged from the preceding frozen checkpoint.

V3 remains isolated pending numerical-kernel and cold-TLS latency benchmarks,
wider sharing/collision patterns, and application
size/retention tradeoffs. No production or donor source edit. The audit-owned
reuse build cache is 3.4 GB under `/tmp`; approximately20 GB of user-quota
headroom remains. No historical cache, evidence, or unrelated data was removed.

## Previous checkpoint: scalar source closure and weak-key proof reuse

All top-level scalar C files are now individually source-read complete: 114 in
archived Calcium and 108 in current FLINT. The new 64-file slice adds 1,721 lines,
plus 33 supporting header-template lines. Total coverage: 296 complete and seven
partial files, not complete donor libraries or their scalar tests. Both pins and
all tracked snapshot hashes verify. An initial coverage entry incorrectly named
generated `flint.h`; verification rejected it. The tracked `flint.h.in` range was
independently read and credited without replacing the pinned inventory.

A new current-native defect concerns random test-data construction:
`ca_randtest_same_nf` omits rational normalization. Of 10,000 seeded four-bit
samples, 2,221 are noncanonical and 107 receive false negative `is_one` answers
(e.g. 2/2). All 10,000 canonicalized-copy controls pass. Memcheck reports zero
errors/all blocks freed; exit1 records mathematical failures. Initial -Werror
compilation caught deprecated probe API names; the corrected probe uses current
names. Archived code has the same mechanism but was not executed. Ordinary
canonical rational arithmetic is not implicated. Hyper's public fraction
constructor already performs exact reduction; no production transfer is needed
for that invariant.

The separate `root-exp-reuse-trial-hyperreal` preserves every v2 proof rule and
all numeric nodes/caches. A 16-entry per-thread cache stores weak pairs and exact
structural-equality results. Pointer identity is checked before lookup; exact pair
keys prevent hash-collision false hits, and weak references prevent address reuse
while an entry exists. A miss, reentrant borrow, or unavailable TLS uses the full
original comparison. This adds no node field or serialized state and no strong
child-graph retention. At most32 Node allocations can remain weakly held per
touched thread; that is not a whole-query bound or a peak-memory measurement.

Debug and release each pass 780 all-feature library tests; eight new tests cover
weak lifetimes, thread exit, eviction/collisions, borrowed-cache fallback,
refinement stability and deep proof outcomes. Clippy lib/tests all features with
warnings denied passes. Deep-fresh public Memcheck passes 100 identity queries,
zero errors/no definite, indirect or possible losses; 1,928 bytes remain reachable.
The earlier v2's MPFR and consumer results are not silently credited to v3.

A non-overlapping three-way confirmation campaign records 1,728 CPU observations
in 24 groups (depths 1/8/32/128, identity/unresolved, fresh/pair/difference). An earlier
1,728-row CPU campaign overlapped Memcheck and is preserved but excluded from
performance conclusions. For depth 128 unresolved pairs, v3 takes 2.640 us versus
v2 9.271 us and baseline 2.214 us: ratio 0.2854 vs v2 (95% 0.2723–0.2862), 1.1944 vs
baseline (95% 1.1574–1.2414). Deep identity pairs take 0.629 us versus v2 7.099 us,
both returning Equal; baseline returns Unknown. Fresh deep cases remain near v2
parity. Shallow unresolved pairs still cost 1.18–1.36x baseline; some retained
differences also regress modestly. This is not a general no-regression result.
Linked CPU driver grows 3,080 bytes over v2 and 11,776 over baseline. Non-test-file
delta versus baseline is 234 lines/9,088 bytes, including test-include directives.
The separate allocation campaign completes 432 observations in 24 groups, with ten
fresh or 100 retained queries per observation. Deep unresolved retained pairs
request 30 allocations/2,392 bytes versus v2 39/11,220 and baseline25/1,768.
Retained unresolved differences remain 1/768 in every variant; fresh deep counts
match v2. These are allocation requests, not live-memory accounting for weakly
held dead Node allocations, and instrumented clocks are excluded from CPU claims.

`root-exp-reuse-experiment.json` binds 20 sources/support files and 60 evidence
files. `verify-root-exp-reuse-checkpoint.mjs` passes all ten chained checkpoints,
including exact 780-test membership and all prior 772 tests, source footprint,
scalar-C read closure, known native failures, every benchmark row/outcome/order,
recomputed medians/intervals, and the explicit overlapping-campaign exclusion.
Formatting and scoped whitespace checks pass. All qualification handles are
terminal. All 176 live Hyperreal source/support files still match the baseline.

**V3 remains isolated and pending further qualification**, including directed-MPFR
and consumer reruns, numeric-kernel workloads, first-touch/high-thread-count TLS
costs, other sharing/collision patterns, and live-memory/size tradeoffs. No live
production edit or donor patch. The full ecosystem audit remains active. New
builds use 647 MB in the audit-owned /tmp reuse target after the user's cleanup;
no previous evidence or unrelated data was removed.

## Previous checkpoint: completed consumer gates and opaque comparison costs

All five paired consumer gates now finish: Hypercurve contributes 1,761 passing
tests per variant across 45 suites, with nine existing ignored tests (six benchmark
drivers and three long stroke regressions). Combined completed consumer gates:
3,309 passed per variant. Library/integration target lists and exact test membership
match. This is not full CI, both-profile consumer coverage or a timing benchmark.

The remaining work-cost concern is measured, not merely hypothetical. An additional
768 CPU and 192 allocation observations cover independently rebuilt nested-sine
atoms at depths 1/8/32/128, exact identities and unresolved perturbations, retained
pairs and retained differences. At depth 128, an unresolved pair costs 4.328x
baseline CPU (95% interval 4.259–4.490): 9.233 us versus 2.160 us. Requested allocations
rise from 25 calls / 1,768 bytes to 39 / 11,220. Retaining its difference instead
reuses Unknown work, with 1.030x CPU and identical one call / 768 bytes. Shallow
identity queries become decidable and faster; the deepest identity becomes
decidable but costs 3.302x.
Different-outcome clocks are not equal-work comparisons. Driver file growth of 8,696
bytes is not whole-application size; requested bytes are not peak heap.

The collector's 128 visits do not cap opaque structural comparisons. Existing
equality uses a 64-visit fast path then memoized node-pair traversal; independent
deep graphs incur that work again for rebuilt differences. **Keep the candidate
isolated pending reuse/scheduling work**, preserving its proof domain. No live
production transfer has been made. All 176 live Hyperreal snapshot files remain
unchanged. No donor patch or external report was submitted.

`sign-work-cost-experiment.json` binds nine source/support and 31 evidence files.
Its verifier passes all nine chained checkpoints, including exact consumer target
and test sets, raw benchmark medians/intervals, and prior known failures. The
earlier immutable partial checkpoint remains unchanged. Full ecosystem and both
donor audits remain incomplete; read coverage is 232 complete and six partial files.

## Previous checkpoint: consumer decisions and serialization

The sign-capable candidate remains isolated. Completed paired all-feature
library/integration gates: Hyperlimit361 debug tests, Hypersolve797 debug,
Hypertri187 debug, Hyperlattice203 release, per variant. Identical test membership
and the773-file consumer snapshots verify. Hypercurve's library suites pass914
tests each with six pre-existing ignored benchmark drivers; full integration
qualification is ongoing and is excluded from this immutable checkpoint.

The new224-query public corpus exercises strict sign, ordering, 2D orientation
and Hypersolve structural classification. Baseline certifies8; the candidate
certifies160, adding152 exact decisions without changing inputs or policy.
All64 beyond-cap controls remain Unknown; no wrong returned sign is observed.
Focused release Memcheck has zero errors/no lost blocks (7,208 reachable bytes).
These are capability checks, not consumer CPU benchmarks.

Fifty-one additional complete donor files /5,025 lines, plus41 documentation
lines, bring this continuation to232 complete and six partial files. Both pins
and all tracked source hashes verify. Two new current-FLINT defects reproduce:

- Export/import loses three formal Arg values under both export flags: six
  losses among48cases, despite successful parsing. The42 controls preserve known
  equality. The documented export contract is stronger than mere parse success.
- Factor insertion updates exponent0 instead of the matched index:48 incorrect
  updates among90 prime-factor cases, with42 passing updates and90 passing
  initial reconstructions.

Both native probes are memory-clean and exit1 for mathematical failures. The
archived source has the same mechanisms but was not executed. No donor patch or
external issue was submitted. Weak representation hashes and dormant Q(i)
denominator-pointer mistakes are recorded separately, not inflated into proved
public numerical failures. No complex API or context-owned lifetime model is
proposed as a replacement for Hyper's finite-real DAG.

`sign-consumer-experiment.json` binds11 source/support and87 evidence files.
`verify-sign-consumer-checkpoint.mjs` passes, including its seven preceding
checkpoints, snapshot/metadata paths, exact test/corpus membership and known
failure dispositions. It does not bind still-running Hypercurve logs or claim
all-profile/full-CI/full-inventory completion. Work-cost qualification of opaque
paired-DAG comparisons and final downstream disposition remain open.

Storage notes and all failed harness/environment attempts are in the root
continuation ledger. `/tmp` has a per-user quota lower than its filesystem free
space. Only an incomplete audit-owned duplicate was removed, after every copied
byte was checked against the preserved original. Sources/evidence and live user
changes remain intact. Existing build targets are reused through recorded links;
the quota-failed build and compiler-cache sandbox failures are preserved.

## Previous checkpoint: nonzero exponential proofs and polynomial evaluation

`root-exp-sign-trial-hyperreal/` is a new isolated candidate; the preceding
zero-only candidate and all its evidence are unchanged. After proving both
operands positive, the proof retains the sign of an exact rational log remainder
when all opaque terms cancel. Both subtraction orientations are tested. No
numerical nodes, caches, dependencies or serialized state change. Collector
limits remain unchanged and do not cap every opaque structural comparison.

All 772 library tests pass in debug/release, including eight focused tests:
the independent 512-case rational oracle, positivity, limits, failed-proof retry,
cache/serialization preservation, and 30 new tiny signed remainders in both
subtraction orders. Clippy with warnings denied passes. The unchanged directed
MPFR oracle passes 15,050 enclosure checks per profile, plus 261 state checks.
The original public corpus still gives 48 Equal identities and 48 correct
NotEqual perturbations. A new 144-query tiny-sign corpus gives baseline 144
Unknown versus trial 48 Positive, 48 Negative, 48 Unknown in both profiles.
The 96 added decisions are 16 formulas times two orientations times three
precision floors. Beyond-limit controls remain Unknown. Both public Memcheck
runs have zero errors and no definite/indirect/possible losses (11,624 and 7,208
reachable bytes). This is not downstream consumer qualification.

The matched CPU6 ABBA/BAAB campaign repeats the unchanged 33 query and 72 numeric
workload groups: 5,040 CPU observations and 1,260 separate allocation observations.

| Workload | Paired CPU sign/baseline | Calls / requested bytes, baseline → sign |
| --- | ---: | --- |
| Fresh separated exponential pair | 0.145 | 108 / 7,760 → 25 / 2,520 |
| Retained separated pair | 0.825 | 6 / 520 → 9 / 1,032 |
| Retained unresolved tiny exponential pair | 1.193 | 25 / 1,768 → 30 / 2,392 |
| Retained unresolved trigonometric exponential pair | 1.234 | 25 / 1,768 → 27 / 1,936 |

Separated pairs return NotEqual in both variants, making these equal-outcome
comparisons. Identity latency comparisons still buy new decisions versus Unknown.
All 72 numeric allocation groups are identical. Numeric hot-operand timings
are near baseline; none of the numeric groups has a bootstrap interval lower
bound above 1.1 in this session. Timings remain host/session evidence, allocation
requests are not peak heap, and linked driver sizes are not application sizes.
Numeric CPU driver +11,272 file bytes; query driver +8,672. Non-test source-file
delta 150 lines / 6,102 bytes includes a test-only wrapper; separate tests add
248 lines / 10,155 bytes.

**Disposition: pending consumer/work-limit qualification, not rejected or
retained in live Hyper.** Completeness improvements have priority over remaining
query overhead, but the consumer effect and opaque-comparison cost still need
evidence. `root-exp-sign-experiment.json` binds 25 source/support files and 101
result artifacts. `verify-root-exp-sign-checkpoint.mjs` validates the original
and new corpora, failures, memory checks, and all four benchmark campaigns,
including recomputed paired medians and bootstrap intervals.

### Additional donor reads and stale polynomial output

Ten full files / 1,992 lines independently read across archived/current trees:
`set_qqbar.c`, `fmpz_mpoly_evaluate.c`, `fmpq_poly_evaluate.c`,
`fmpz_poly_evaluate.c`, and `fmpz_mpoly_q_evaluate.c`. Coverage totals 181 complete
and six partial files; neither donor nor full ecosystem is complete.

Quadratic import performs bounded partial square-factor extraction and exact
embedding selection; its result is not necessarily squarefree-normalized.
Generic sparse Horner evaluation uses an explicit stack, variable/power heuristic,
and variable-by-term scratch storage. Rational-function evaluation keeps checked
division distinct from the caller-preconditioned no-zero variant. Hyper's fused
dense tensor-axis and rational homogeneous Horner paths already avoid several
donor costs; sparse scheduling is not demonstrated to improve those workloads.

New confirmed current-FLINT defect: non-rational `ca_fmpz_poly_evaluate` computes
into a temporary and clears it without assigning the output. The independent
64-case polynomial probe reports 24 stale-output failures: linear/quadratic
polynomials at sqrt(2), pi and 1+i, each with zero/seven/Unknown/in-place output.
Zero/constant-polynomial and rational-input controls pass. The rational-polynomial
companion passes all 64 cases against separately constructed scalar formulas.
Memcheck is clean with all blocks freed; exit 1 records mathematical failure.
Archived source has the same omission, but only current FLINT was executed.
No donor patch, external report or Hyper production mutation was made.

## Phase and conversion boundary checkpoint

Twenty-one further complete source reads cover floor/ceil, binary64 and complex
import, conjugation, real/imaginary projections, phase/sign, root-of-unity phase,
and current arf conversion support. Along with 218 supporting documentation/header
lines, this adds 2,216 read lines. Overall coverage is now 171 complete and six
partial files, not donor-library or ecosystem completion.

The independent `flint-scalar-boundary-probe.c` confirms a current-FLINT principal
phase defect: `ca_arg(-1)`, `ca_arg(-2)`, `ca_arg(-1/3)` and `ca_arg(-infinity)`
return -pi instead of +pi. The documented range is (-pi,pi]; the negative sqrt(2)
algebraic path correctly returns +pi. Thirty separate/in-place phase controls
include eight failures and 22 passes. Archived source has the same mechanism but
was not executed. No donor patch or external issue was submitted.

The probe also passes 10,013 binary64 conversions checked against GMP exact
rationals or explicit exceptional-state expectations. All tested arf values use
no allocated storage, supporting the omitted `arf_clear` on this bounded import
path, not a general cleanup exemption. Eighty-four simple dyadic real/complex
floor/ceil controls pass. Memcheck has zero errors and all blocks freed; native
and Memcheck commands both exit 1 because of the eight mathematical failures.
This is qualification, not a performance comparison.

Complex floor/ceiling intentionally round the real part; their formal-generator
fallback does not provide Hyper's certified integer-return contract. Conjugation
conservatively checks branch-cut exclusion before pushing inside functions and
reevaluates coefficients if a transformed generator changes algebraic carrier.
Hyper already has exact IEEE-to-dyadic conversion and +pi negative-axis behavior;
its checked atan2 preserves uncertain branch decisions as Exhausted. The existing
negative-axis regression test passes in both frozen baseline profiles.

`scalar-boundary-experiment.json` binds this probe and its nine result artifacts;
`verify-scalar-boundary-checkpoint.mjs` validates the full phase membership and
expected failures, conversion/rounding counts, Memcheck, source-read coverage and
the earlier checkpoint chain. The proof-only candidate below is still pending.

## Proof-only exponential relation checkpoint

The separate `root-exp-proof-trial-hyperreal/` candidate preserves numerical
nodes and caches while proving logarithmic linear relations between positive
exponential expressions. It supports roots, squares, products, inverses and
binary scaling only when every leaf establishes positivity. Linear exponent
coefficients are exact rationals; opaque terms cancel only by existing structural
equality. There is no numerical relation guess, new approximation variant, cache
field or serialized field. Failed structural work shares the existing exact-sign
Unknown cache, without preventing later numerical separation.

Collector limits are 128 visits, 16 terms and 1,024-bit coefficient components;
linear binary offsets are at most 1,023 in magnitude. These do not bound arbitrary
descendant work performed by existing opaque structural equality. The current
form proves zero only: when opaque terms cancel but a nonzero rational constant
remains, the ordinary sign/refinement fallback is still used. A sign extension
must handle both subtraction orientations and retain genuine unresolved controls.

Qualification completed for this isolated candidate:

- 48 Equal identities and 48 correct NotEqual perturbations in both profiles;
  baseline has 12 Equal / 36 Unknown identities. These remain 16 signed formulas
  at three budgets, not 48 independent formulas.
- 770 all-feature library tests pass in debug and release; six added tests cover
  512 independently calculated rational-log coefficient cases and unequal
  controls, positivity restrictions, failed-proof cache/retry, hot operand/root
  cache preservation, parser limits, and serialization without serialized facts.
- Each profile passes the unchanged 15,050-check directed-MPFR oracle and the
  261-check state corpus. Earlier baseline results use the identical frozen
  oracle and input workload. Clippy `--lib --tests --all-features -- -D warnings`
  passes. This does not claim the entire downstream CI matrix was run.
- Public Memcheck has zero errors, zero definite/indirect/possible losses, and
  11,624 still-reachable bytes. Initial prototype Rational AddAssign and
  query-harness reciprocal-method compile failures remain recorded; neither
  is misclassified as a baseline numerical failure.

The new query harness tests 11 cases across fresh, retained-pair and
retained-difference lifecycles; the unchanged numeric harness tests 12 cases
across six lifecycles. CPU6 ABBA/BAAB measurements contain 5,040 CPU observations
and 1,260 separate allocation observations. Raw outcomes are checked: the five
identity families must change from Unknown to Equal; unresolved controls must
remain Unknown in both variants, and separated controls must remain NotEqual.

| Workload | Paired CPU proof/baseline | Allocation calls / cumulative requested bytes, baseline → proof |
| --- | ---: | --- |
| Fresh root/exp identity, sqrt(2) argument | 0.021 | 853 / 41,280 → 18 / 1,712 |
| Retained pair, same identity | 0.294 | 25 / 1,768 → 9 / 1,032 |
| Retained pair, unresolved tiny exponential difference | 1.203 | 25 / 1,768 → 30 / 2,392 |
| Retained pair, unresolved trigonometric exponential identity | 1.303 | 25 / 1,768 → 27 / 1,936 |
| Retained pair, separated exponential difference | 1.553 | 6 / 520 → 11 / 1,144 |

Retaining the difference itself reuses its failed-proof cache; unresolved controls
are near baseline. All 72 numeric groups have identical allocation counts and
requested bytes. Numeric hot-operand timings are roughly near baseline, with
sin(1) at 1.082 (descriptive bootstrap interval 1.014–1.133), not the earlier
constructor rewrite's multi-fold regression. No numeric group's interval lower
bound exceeds 1.1 in this run. This is scoped host/session evidence, not a
whole-library no-regression claim. Different-outcome timing is not equal-work
speedup, and cumulative allocation requests do not measure peak/retained heap.

The numeric CPU driver grows 11,344 file bytes (text +9,544, data +64, bss -1,408);
the query CPU driver grows 8,776 file bytes. These are linked drivers, not stripped
downstream applications. Source delta: 143 production lines / 5,782 bytes plus
172 test lines / 6,969 bytes. Two existing files change and two private files are
added, entirely in the isolated copy.

**Disposition: pending, not rejected and not retained.** The additional exact
decisions have priority over performance under this audit's acceptance order.
Remaining work includes nonzero-sign recovery, query scheduling and opaque-work
limits, then proportional downstream qualification. No live Hyper source or
historical archive was changed; all 176 live baseline files still match.

### Additional complete donor reads

Both archived/current versions of `can_evaluate_qqbar.c`, `get_qqbar.c`,
`check_is_algebraic.c`, `check_is_rational.c`, `check_is_integer.c` and `check_ge.c`
were independently read completely: 12 files / 1,981 lines. Coverage is now
71 complete + one partial archived file (11,595 read lines) and 79 complete +
four partial current files (12,643 read lines). This does not complete either
source tree or the ecosystem inventory.

Conversion capability skips unused generators despite a stale TODO; algebraicity
checking conservatively examines all generators. Neither a failed conversion nor
a failed algebraicity proof establishes transcendence or irrationality. Generic
conversion borrows cached algebraic values, tracks owned temporaries separately,
and selectively caches square-root conversions. Degree/height caps are not
uniform across every route. Rationality checking cannot numerically exclude a
real from dense Q; integrality can be disproved by an enclosure containing no
integer. Ordering first establishes realness, separates special-state semantics,
then uses enclosures/algebraic conversion/equality. Hyper already exploits the
certified sign of subtraction, a donor TODO rather than a missing Hyper feature.

## Pinned sources and coverage

- Standalone Calcium: `8dbb16fc4fe92eaf3ebbc7478d629e994d39f944` (2023-11-15).
- Current FLINT: `e269d38061d7a42070ddcffe6eb114466ed4aa7e` (2026-09-07).
- Both repositories are external dependencies under workspace
  `exact-real-references/`; neither has tracked-file modifications.
- `inventory.json` records every tracked file's hash, byte count and physical
  text-line count. Standalone: 684 files / 98,869 text lines. FLINT: 10,352
  tracked paths, with an initial 704-file / 81,687-line exact-number slice.
  Supporting slices may grow; unselected files are **not** counted as read.
- `coverage.json` records 87 standalone files / 13,485 read lines (86 complete,
  one partial), and 100 current-FLINT files / 14,961 read lines (95 complete files
  and five precisely bounded partial reads). Both `ca_ext/` and `ca_field/` directories are completely
  read in both snapshots. Supporting generic-ring and monomial-context reads do
  not credit their unread parent subsystems. No unread range receives credit.
- The [official introduction](https://flintlib.org/doc/introduction_calcium.html)
  and [archived project page](https://github.com/flintlib/calcium) were reviewed.
  The latter confirms that development moved into FLINT in 2023. Documentation
  claims are checked against code, not accepted as qualification evidence.

## Source findings

1. **Representation and lifetime.** The historical `ca.h`, `ca_ext.h` and
   `ca_field.h` describe rational, embedded algebraic-field and multivariate
   rational-function carriers. Extension generators are interned in a context;
   fields share generator identities and hold reduction ideals. Cached balls
   and algebraic embeddings belong to those generators. Context construction
   creates Q and Q(i), and interned objects live until context teardown.
   Hyper instead combines rational/symbolic classes with immutable shared nodes
   and node-owned synchronized approximation caches. A context-wide field
   engine is not yet justified as a replacement; lifetime and allocation costs
   require matched consumer workloads before any transfer disposition.

2. **Zero tests and denominator validity.** Both `ca/check_is_zero.c` versions
   stage exact carrier checks, numeric nonzero exclusion, algebraic evaluation,
   complex normal-form rewriting, and numerator factorization. A ball containing
   zero never alone proves zero. Numerator-only testing relies on a valid field
   denominator. Historical `ca/inv.c` confirms that public inversion requires a
   proven nonzero input; an unresolved input yields Unknown. Its explicitly
   unchecked internal variant has a stronger caller precondition. This is not
   permission to simplify an unresolved Hyper quotient as if its denominator
   were valid. Hyper's algebraic-separation parser likewise admits an inverse
   only after a nonzero certificate.

3. **Checked relation discovery.** Both `ca_field/build_ideal.c` files were
   independently read completely. LLL proposes small integer relations but its
   ball overlap is only a heuristic filter. Log relations are accepted only
   after proving the product of argument powers is exactly one and bounding the
   logarithmic sum in absolute value below 2, excluding any nonzero multiple of
   2*pi*i. Multiplicative relations likewise require exact algebraic arithmetic
   or a proved logarithmic identity. Relations for radicals and Vieta formulas
   are exact polynomial consequences. Bounded Groebner computation controls
   complexity, not mathematical truth. A bounded, demand-time log-identity
   certificate is a concrete Hyper candidate; importing LLL or generic field
   construction is not assumed necessary.

4. **Cache reentrancy.** Historical field-cache insertion saves the allocated
   field pointer before building its ideal, since recursive relation work may
   grow the pointer table. This is a sound ownership detail, but not by itself
   a reason to replace Hyper's Arc-owned nodes. Extension and field caches
   allocate/rehash before searching for an existing entry; performance/memory
   tradeoffs remain unqualified.

5. **Special-function congruences.** Historical `build_ideal_erf.c` inserts
   complement, oddness and imaginary-argument relations only after proving
   equality of the arguments. `build_ideal_gamma.c` proves small integer
   argument shifts and inserts recurrence identities when the coefficient field
   embeds in the current field. Compare real-domain erf/erfc complement and
   non-half-integer gamma recurrences with Hyper before deciding transfer.

6. **Current-version lifetime questions resolved.** The generic qqbar context
   consists of inline real-only/degree/bit-limit options and inherits no-op generic
   cleanup. A resource-limit jump past `gr_ctx_clear` therefore does not leak that
   context in this pinned implementation. Likewise Calcium's polynomial contexts
   initialize inline monomial-order/packing tables; their skipped clear routine
   is a no-op. Exact supporting ranges are in `coverage.json`. Neither conclusion
   qualifies other generic-ring methods or older uninspected backend versions.

7. **Independent current field/extension reads.** The current implementation still
   grows/rehashes caches before an intern hit is known, constructs temporary number
   fields eagerly, and keeps interned objects until context teardown. Reverse
   extension destruction preserves dependency lifetimes. Representation ordering
   uses polynomial/embedding and depth/opcode/argument order, not arbitrary real
   comparison. The enclosure cache records working precision, not achieved accuracy;
   Cbrt/Root raw dispatcher cases remain explicitly absent. Existing interning tests
   cover representation/pointer roundtrips rather than semantic completeness or
   concurrency. These mechanisms do not demonstrate a better cache owner for Hyper.

## Confirmed Hyper completeness gap

Independent exact oracle, with positive integer `a` and positive nonsquare `d`:

`u = a + sqrt(d)`, `v = a*a + d + 2*a*sqrt(d)`.

Algebra establishes `u*u = v`; positivity then establishes `2*ln(u) = ln(v)`.
Strict monotonicity proves `2*ln(u) != ln(v + 1/1024)`. A separate donor-only
control checks that `2*Log(-u)` differs from `Log(v)` by `2*pi*i`: the product
identity alone must not erase the principal complex logarithm's branch.

The corpus uses a = 1, 2, 3 and d = 2, 3, 5, 6, 7, 10, 11, 13, 17, 19:

| Request | Frozen Hyper debug | Frozen Hyper release | Current FLINT |
| --- | ---: | ---: | ---: |
| Algebraic square controls | 90 Equal | 90 Equal | 90 Equal |
| Log identities | 90 Unknown | 90 Unknown | 90 Equal |
| Perturbed controls | 90 NotEqual | 90 NotEqual | 90 NotEqual |
| Complex branch controls | not a Hyper real-domain request | not a Hyper real-domain request | 30 NotEqual |

There are 30 distinct formulas, each real-domain request repeated at three
budgets. These are not 90 independent formulas. Hyper's minimum precisions are
-64/-256/-512; FLINT uses numeric precision-limit options 64/256/512. Their
construction, symbolic work, cache histories and budget semantics differ.
FLINT's options are set for the queries after initial construction. These
results establish a capability difference, **not** matched precision-cost or
speed claims. Clocks in the diagnostic CSVs also differ (wall versus CPU).

`baseline-hyperreal/` freezes all 176 tracked/untracked nonignored files from
Hyperreal HEAD `da26e961bf76adf13829d8a26ced6cdaacb6b564`, including the existing
uncommitted work. `baseline-hyperreal.json` records exact content hashes;
`baseline-hyperreal.patch` records the tracked delta. None of those changes is
attributed to this continuation. The probe now depends on the frozen source.

Hyper's current `Real::ln` retains rational, pure radical, base-2/base-10 power
and exponential identities, but falls back to a generic computable for these
composite positive algebraic arguments. Its generic logarithm construction
then performs binary/root range reduction. Existing algebraic norm certificates
can prove the underlying polynomial equality, but do not prove the logarithmic
relation. This matches the independent probe rather than assuming a gap from
source inspection alone.

## Executed qualification

- Native FLINT: `autoreconf -i`, then
  `./configure --disable-static --enable-assert CFLAGS='-O2 -g0'`, `make -j8`.
  Build passed in about 80 seconds. GCC reported warnings in other FLINT modules;
  a successful build does not certify those warnings harmless. Full logs retained.
- Unchanged `make -j4 check MOD=ca`: all 29 scalar test functions passed.
- Unchanged `make -j4 check 'MOD=ca_ext ca_field qqbar fmpz_mpoly_q'`:
  all 73 supporting test functions passed. Running tests does not credit reading
  their source files or prove the entire library correct.
- The independent C probe builds with `-Wall -Wextra -Werror`; donor headers
  are system includes. First link attempt used the wrong `.libs` location;
  corrected command links the actual build-root library. Both attempts retained.
- Frozen Hyper debug/release probes: 270 checked rows each; native donor: 300.
- Both focused Memcheck runs: zero errors and no definite, indirect or possible
  leaks. Native FLINT frees all blocks; the frozen Hyper process retains 20,656
  reachable bytes. Different work/formatting/cache lifetimes prevent interpreting
  these aggregate counts as an allocation A/B benchmark.
- Initial live-tree probes also reproduced the same decisions. Initial frozen
  builds failed because the first snapshot omitted Cargo-declared bench files;
  the snapshot was extended after checking unchanged HEAD/diff/source hashes.
  The successful complete-snapshot runs are authoritative.
- Four audit JavaScript programs pass syntax checks; inventory verification
  validates pinned files and nonoverlapping in-bounds coverage ranges.
- `verify-results.mjs` verifies frozen source hashes, full unique Cartesian
  corpus membership, exact outcome counts, native test completions and Memcheck
  success. It does not pretend to independently prove that a human read a line.

## Reproduction

From `exactcore-hyper-comparison`:

```sh
node audits/continuation/calcium/inventory.mjs verify
node audits/continuation/calcium/verify-results.mjs
node audits/continuation/calcium/verify-checkpoint.mjs
node audits/continuation/calcium/verify-root-exp-checkpoint.mjs
node audits/continuation/calcium/verify-scalar-boundary-checkpoint.mjs
node audits/continuation/calcium/verify-root-exp-sign-checkpoint.mjs
cargo run --offline --manifest-path audits/continuation/calcium/hyper-probe/Cargo.toml
```

Build/test commands, cwd, return status, and complete stdout/stderr are retained
under `results/`. `capture.mjs` requires a fresh result tag and refuses replacing
an earlier log. Compiler outputs are external; no system installation was done.
Donor source remains external with its original LGPL license. The independently
written probes do not copy Calcium implementation code.

## Isolated log prototypes: qualified findings, not retained

The original v1 zero-only prototype and the revised v2 sign prototype are preserved
as `results/log-trial-v1-source-diff.stdout` and
`results/log-trial-v2-source-diff.stdout` (git diff exit 1 means differences exist).
The trial modifies only the isolated baseline's node include and sign-query hook,
plus two new private implementation/test files. Live Hyper remains untouched.

V2 proves positivity of every recovered argument, clears rational coefficients by
a positive common denominator D, and constructs positive products P and N. Then
`D * sum(c_i*ln(a_i)) = ln(P/N)`, whose sign equals `sign(P-N)`. An existing
algebraic certificate is required before recursively querying the product
difference; this also excludes log-proof recursion. Limits are 128 collector
visits, 16 log terms, 8-bit coefficient components, denominator LCM at most 256,
per-term exponent at most 64 and total exponent at most 256. Cancellation is checked
before work and before publishing a result. There is no numerical relation guess,
new node field, serialized cache or new public API.

- V1: 270 public rows pass in debug/release, with 90 log Equal results instead of
  baseline Unknown; 771 debug library tests pass. The first 768-observation paired
  campaign found a 35x repeated-unresolved rational-log regression.
- V2: 270 public rows pass in debug/release, all 773 library tests pass in both
  profiles versus 764 baseline tests, and nine focused tests cover positivity,
  invalid arguments, limits, roots/products/quotients, rational coefficients,
  tiny perturbations, query order, concurrency, cancellation and serialization.
  One test checks 2,000 independently computed BigRational products, requires at
  least 1,500 direct proof decisions, and verifies all 2,000 public sign results.
- V2 standalone public Memcheck: zero errors and zero definite/indirect/possible
  leaks. The separate nine-test runner reports one 48-byte possible loss in
  `std::thread::Thread::new` through the Rust test-runner channel. An unchanged
  baseline single-test runner reproduces that stack and status 97. The raw failure
  is preserved, not suppressed or reported as a clean test-runner gate.

The expanded CPU6 ABBA/BAAB campaign has 1,056 observations (11 cases, fresh/shared
lifecycles, 12 blocks, two observations per variant per block). Allocation builds
are separate, with 264 observations and explicit instrumentation checks. Raw rows,
binary hashes/sizes, outcomes and descriptive bootstrap intervals are retained in
`log-paired-v2-expanded-{cpu,alloc}-summary.json` and `results/`.

| Workload | Outcome baseline / trial | Paired CPU trial/baseline | Allocation calls and requested bytes per query, baseline / trial |
| --- | --- | ---: | --- |
| Fresh log identity | Unknown / Equal | 0.211 | 2,103 / 377 calls; 104,288 / 20,144 bytes |
| Shared log identity via public equality | Unknown / Equal | 2.050 | 36 / 98 calls; 2,448 / 11,256 bytes |
| Fresh near-nonzero rational log difference | Positive / Positive | 0.629 | 473 / 315 calls; 16,832 / 18,624 bytes |
| Shared unresolved multiradical log difference | Unknown / Unknown | 170.931 | 1 / 324 calls; 768 / 32,008 bytes |
| Shared unresolved transcendental log difference | Unknown / Unknown | 5.058 | 1 / 16 calls; 768 / 2,936 bytes |

The tiny rational and correlated quadratic log differences at a 128-bit floor
gain positive decisions and thereafter use the exact-sign cache. The deliberately
expanded multiradical and transcendental controls still remain Unknown, exposing
the cost of rebuilding an unsuccessful proof. Several nanosecond-scale ordinary
controls have mixed small regressions; no blanket no-regression claim is made.
The CPU driver grows 8,432 file bytes (not a stripped application measurement).
Different-outcome ratios are latency observations, **not equal-work speedups**.
Requested allocation bytes are cumulative, not peak heap or retained memory.

**Disposition:** the unconditional late sign-query hook is not selected as
implemented. The bounded exact proof remains a promising completeness idea, but
history-safe scheduling/reuse and broader consumer costs need a separate design
and qualification. No production transfer or full downstream qualification is
claimed for either prototype.

## Error-function relation follow-up

The current ideal builder proves equal/opposite arguments before inserting erf,
erfc and erfi relations. Hyper has separate cached-complement `Erfc` nodes, while `erf`
is an expanded series/Gaussian product. Its general gamma evaluation is explicitly
limited to integer/half-integer inputs; arbitrary-argument gamma recurrence would
first require a certified kernel, not merely a symbolic rewrite.

Independent public Rust/C probes test erf complement, erf oddness and erfc
reflection at `-2, -1/3, 0, 1/3, 2, sqrt(2), pi`, each at three recorded budgets,
with separate `1/1024` perturbations. Every construction is fresh per case/control.
There are 21 identity formulas (18 nonzero-argument formulas), not 63 independent
formulas. These are capability probes, not cross-library performance measurements.

| Corpus, 126 rows per build | Equal identities | Unknown identities | Correct NotEqual perturbations |
| --- | ---: | ---: | ---: |
| Frozen Hyper debug | 9 | 54 | 63 |
| Frozen Hyper release | 9 | 54 | 63 |
| Current FLINT | 63 | 0 | 63 |

The native probe also passes Memcheck with all blocks freed and zero errors.
This establishes a separate completeness candidate; no error-function production
change had been attempted at that checkpoint. Correction from the subsequent
kernel read: `Erfc` rebuilds `1 - erf_expanded(x)`; it is not an independent
asymptotic stable-tail kernel. The documented absolute-error contract and the
cost of cancellation/refinement still need qualification.

## Scalar continuation and a native Gamma counterexample

Both snapshots' equality, realness, representation-equality, error-function,
Gamma, complex-normal-form, exp/log, field-merge, storage-transition and demotion
files are now completely read. New supporting truth-enum header ranges are
recorded separately; the rest of those headers is not credited.

The equality/realness stages reinforce an existing Hyper design: exact carrier
and representation facts first, directed separation where decisive, and explicit
Unknown when a deeper proof is inconclusive. Neither overlapping balls nor
matching expression hashes alone prove equality. Context interning makes exact
field-pointer comparisons meaningful inside the donor; it is not a general real
comparison technique. Complex exp/log/power rewrites carry branch corrections
and nonzero preconditions. Hyper's positive-real domain already avoids that
complex-branch machinery; no unconditional complex identity was transferred.

Field merging uses sorted generator unions, three temporary arrays and exact
variable substitution. Shallow coefficient views avoid intermediate big-integer
copies. Demotion swaps owned coefficients into rational or single-generator
number-field values, but does not evict context-owned fields. General field
shrinking and compatible cross-field storage reuse are still TODOs. This is not
evidence that replacing Hyper's node ownership would save memory.

`flint-gamma-special-probe.c` independently exposes an actual defect in current
`src/ca/gamma.c`: `ca_check_is_*` predicates are used as ordinary C booleans,
although the enum encodes true as zero and false as one. All seven special-value
controls fail: positive infinity becomes undefined, while negative infinity,
Unknown, undefined, unsigned infinity and both imaginary infinities become
positive infinity. The expected imaginary-infinity result follows the donor's
documented branch intent. Three independent finite controls pass:
Gamma(3)=2, Gamma(1/2)=sqrt(pi), Gamma(-1/2)=-2sqrt(pi).

The archived source has the same boolean-use defect; only the pinned current
FLINT build was executed. `gamma-special-native` and `gamma-special-memcheck`
exit 1 for these mathematical counterexamples, not a harness/build failure.
Memcheck reports zero errors and all blocks freed. No donor patch or external
issue was created. The transferable lesson is explicit known-true handling;
Hyper already uses conservative Rust enum/Option boundaries.

## Isolated erf representation experiment — not selected

`erf-trial-hyperreal/` starts from the same hash-verified 176-file baseline as
the log trial but does **not** contain the log prototype. Only `algebra.rs` and
`statistics.rs` differ. Public `erf` becomes a canonical complement of `Erfc`,
explicit negative operands use oddness/reflection, and the Erfc dispatcher calls
the original expanded erf constructor through a private helper to avoid recursion.
A symmetric bounded nested-add rule recognizes `x + (c - x)`.
No public node variant, cache field, serialized format field or dependency is added.

The first representation-only attempt improved identity queries from 9 Equal /
54 Unknown to 33 Equal / 30 Unknown. Adding the nested cancellation rule closes
all 63 identity queries; all 63 perturbed controls remain NotEqual. The final
public corpus passes in debug/release and focused Memcheck (zero errors, no
definite/indirect/possible losses; 4,208 bytes remain reachable in globals/runtime).
These are three identities at seven arguments and three budgets, not 63 unrelated
mathematical formulas.

Independent qualification shared by baseline and trial:

- 5,856 directed-MPFR absolute-error enclosure checks in each profile: 160
  inputs, erf and erfc, fresh/mixed/serialized graphs, coarse-to-fine-to-coarse
  requests, tiny inputs down to 2^-1024 and tails through +/-32. Deep requests
  reach binary precision -1600. MPFR runs at 2,304 bits; increasing erf and
  decreasing erfc correctly propagate directed input endpoints. This is an
  absolute one-ulp contract check, not a claim about fixed-precision relative tails.
- 270 enclosure checks per profile cover abort/retry, four-thread shared-cache
  refinement and non-rationally represented zero inputs. Three additional sign
  queries on nearby distinct arguments reject a false structural cancellation.
- All 764 existing all-feature library tests pass in debug and release; the
  earlier serde-only run passes 693 tests. These existing tests did not expose
  the new completeness regression below.
- The initial MPFR harness incorrectly imported Hyper's unsigned numerator
  without its separate sign, failing both builds immediately. It was corrected,
  and the original failure logs are retained. No Hyper numerical defect is
  claimed from that harness error.

The first retention failure is completeness: four negative/reflected Erfc
expressions keep a positive sign only in a nonserialized cache. After serde,
their generic `2 - Erfc(x)` form no longer proves positivity structurally.
The baseline's named Erfc node preserves all four positive facts. `erf-trial-facts`
deliberately exits 101 for this regression; no test was weakened or suppressed.

The second failure is repeated-refinement cost. Twelve paired ABBA/BAAB blocks
give 2,304 CPU observations across 12 cases and four lifecycles; separate
instrumented runs give 576 allocation observations. Measurements are pinned to
CPU 6, with matched iteration counts per group. The refine lifecycle constructs
a fresh graph and requests -32, -128, then -512; warm is a retained -256 result.
Each process preconditions fresh and retained -256 queries eight times, so the
refinement workload includes any first deeper global-constant warmup. It is not
a separate fully cold-process or steady-state global-cache benchmark.

| Refinement workload | Paired trial/baseline time | Requested allocation bytes, baseline → trial |
| --- | ---: | ---: |
| erf(1/3) | 1.542x | 66,626 → 96,158 |
| erf(-1/3) | 1.559x | 67,154 → 97,246 |
| erf(2^-512) | 2.479x | 2,904 → 10,260 |
| erf(8) | 1.481x | 418,739 → 616,706 |
| normal CDF(1/3) | 1.525x | 64,919 → 94,098 |

The private expanded arithmetic is reconstructed through Erfc on every deeper
request, losing the baseline erf graph's intermediate cache reuse. Construction
of nonzero erf can be cheaper, and the exact complement now reduces before
approximation; those gains do not justify the failed completeness and refinement
gates. Ordinary-add/sqrt controls are near parity in this session. Even exact-zero
erf construction regresses (3.8x), although an early zero guard could separately
remove that small absolute cost. It would not fix the principal failures.

The matched linked CPU driver grows 2,064 file bytes (text +1,448, data unchanged,
BSS -1,440); this is not a stripped downstream application measurement. Changed
library source grows 50 lines / 2,286 bytes. Allocation bytes are cumulative
requested storage, not peak heap; instrumented clocks are not CPU evidence.
Bootstrap intervals are descriptive of this host/session, not universal bounds.

Disposition: **no erf production transfer retained**. Keep the independent
qualification probes and exact counterexamples. Preserving named relations while
retaining kernel caches and serialized range facts needs a different design.
The general nested-add cancellation rule remains independently unqualified; this
combined experiment does not establish its standalone payoff. No downstream
matrix/geometry/solver gate is claimed for a prototype already rejected at scalar
qualification. Source hashes and explicit failed gates are in `erf-experiment.json`;
`verify-erf-checkpoint.mjs` verifies this evidence and the preceding checkpoint.

## Arithmetic and root/exponential checkpoint

The next 40 fully read source/test files cover both versions of add/subtract,
multiply/divide, negation/dot, square-root construction/factor extraction, powers,
factorization, enclosure evaluation, absolute/complex sign, initialization,
positive-infinity checking, trig construction and supporting tests; the current
inverse and archived log-identity test fill earlier asymmetric coverage. Exact
paths and ranges are in `coverage.json`; supporting documentation remains partial.

Generic arithmetic allocates per-ideal quotient arrays and reduces numerator
and denominator, recanonicalizing if reduction changed them. Rational and
same-field paths avoid much of this work. Monomial-denominator simplification is
selective, not a cheap complete normal form. Number-field factorization remains
TODO. Smooth integer factoring is a decomposition heuristic, not a guarantee of
prime factors. Hyper's fused rational/dyadic dot and aggregate paths already do
more than Calcium's simple sequential strided multiply/add loop.

Square-root factor extraction proves an exact two-candidate identity before
using ball separation to choose the principal branch. It never infers equality
from overlapping balls alone. Degree and double-valued expression-size heuristics
select eager versus formal roots/powers; they are not proof inputs. The numerical
enclosure loop can return less than requested relative accuracy at its cap, as
the documented caller contract allows. An exact-zero checkpoint keyed to a single
working precision can be skipped when the cap is off the doubling grid. Raw
generic evaluation allocates a ball vector and evaluates every field generator;
field condensation does not guarantee that every remaining generator is needed.

### Independent identity corpus and isolated trial

For every finite real x, positivity establishes `sqrt(exp(x)) = exp(x/2)`.
The public corpus uses 1/3, sqrt(2), sin(1), pi-3, ln(2)+ln(3), sqrt(2)+sqrt(3),
32*sqrt(2), and sqrt(2)/1024, each with both signs and at three budgets. A separate
right-hand side perturbed by 1/1024 must be unequal. Both operands are built from
separate factories afresh for each query.

| 48 identity / 48 perturbation queries | Equal identities | Unknown identities | Correct NotEqual controls |
| --- | ---: | ---: | ---: |
| Frozen Hyper debug and release, each | 12 | 36 | 48 |
| Current native FLINT | 48 | 0 | 48 |
| Isolated Hyper trial debug and release, each | 48 | 0 | 48 |

There are 16 signed formulas, not 48 distinct formulas. FLINT's precision-limit
option and Hyper's minimum requested precision are not equivalent work budgets;
these are capability probes, not inter-library speed comparisons.

The trial changes only `root-exp-trial-hyperreal/src/computable/node/roots_inverse_hyperbolic.rs`:
25 added lines / 1,394 bytes. Direct `PrescaledExp(x)` becomes `exp(x/2)` under
sqrt. A binary offset of an exponential restores `x` from visible range reduction,
or exactly reconstructs the exponent with k*ln(2), before halving it. All other
sqrt cases remain unchanged. No log or erf trial changes are included.

The first direct-node-only attempt proved 30 identities but left 18 Unknown;
its diff/results are preserved. The complete offset-aware trial proves all 48.
Initial harness compile failures (`sin().unwrap()`, private `shift_left`) and the
trial's initial i32-versus-i64 constructor mismatch are retained under separate
tags; none is a numerical defect. Public multiplication supplies binary scaling
in the corrected external oracle harness.

### Qualification and tradeoffs

- All 764 existing all-feature library tests pass for the trial in both profiles.
- Both variants and profiles pass 15,050 independent directed-MPFR one-ulp
  enclosure checks: 43 exact dyadics, optional sine composition, seven binary
  scales, five cache/serde lifecycles, five precisions. Inputs include ±2^-1024
  and ±128, scales range -31..31, requests range 0..-512. Every input is asserted
  exactly representable before using sine at the two directed endpoints.
- Each variant/profile also passes 261 enclosure checks covering cancellation,
  retry, concurrent shared refinement and sin²+cos²-1 as a non-rational zero
  exponent. Tested structural positive facts survive input/output serialization.
- Focused native identity and public trial Memcheck runs have zero errors and
  no definite/indirect/possible loss. Native frees all blocks; Hyper retains
  reachable runtime/global-cache storage. These are not matched memory benchmarks.
- Matched CPU benchmarks: 12 ABBA/BAAB blocks, 3,456 observations, 12 cases and
  six lifecycles, CPU 6. Separate allocation builds give 864 observations.
  The hot-operand case first computes exp(x) to -768 outside timing, then clones
  it and constructs/approximates a fresh root to -256 per iteration. Warm-root
  reuses the already-computed result instead; refine requests -32/-128/-512 on
  each fresh graph. Construction-only and ordinary-root controls are included.

| Exponent | Fresh trial/baseline time | Hot-operand trial/baseline time | Hot-operand requested bytes, baseline → trial |
| --- | ---: | ---: | ---: |
| 1/3 | 0.365x | 5.139x | 1,016 → 4,192 |
| sqrt(2) | 0.429x | 4.545x | 1,016 → 4,104 |
| sin(1) | 0.493x | 6.801x | 1,016 → 5,360 |
| 32*sqrt(2) | 0.461x | 7.610x | 1,088 → 7,224 |

Fresh and progressively refined numerical evaluation improves in these cases;
retained warm roots are near parity. Tiny/opaque-zero exponents can also improve
the hot-operand workload. However, the ordinary nontrivial hot-operand cases lose
the original exponential result cache. Large-exponent root construction alone
rises from about 22 ns to 1.27 us (56.7x); allocation requests rise from one / 72
bytes to 21 / 1,656. The matched CPU driver grows 1,056 file bytes (text +1,004,
data unchanged, BSS -1,008). This is linked-driver size, not a stripped consumer
application or full-workspace size result. Allocation bytes are cumulative, not
peak heap; instrumented timing is excluded from CPU conclusions. Bootstrap
intervals describe only this host/session. Fully cold process behavior is not
separately measured; global constants receive fixed preconditioning.

Disposition: **do not transfer this eager constructor rewrite as implemented**.
The completeness gain and fresh-evaluation win are real, but the known cache loss
is substantial. Preserve the independent corpus and qualification code. A design
separating semantic identity recognition from the numeric evaluation graph, or
retaining both evaluation routes without large node/constructor costs, remains
open. No downstream geometry/solver test or broad performance claim is made for
this rejected scalar prototype.

`root-exp-experiment.json` binds 20 source/support hashes and 103 raw/summary
artifacts. `verify-root-exp-checkpoint.mjs` validates the isolated footprint,
entire corpus, known failures, oracle/test completions and benchmark membership;
it recomputes CPU/allocation medians from the raw observations. It also imports
the earlier erf/log verifiers. Direct execution passes. Two nested sandbox
captures returned empty stdout despite status 0 and are not accepted as evidence
of completed verification. The approved `verify-root-exp-checkpoint-unsandboxed`
capture contains all four expected completion records and passes a separate
record-count check. No failed or empty capture was overwritten. A final read-only
hash comparison confirms all 176 live Hyperreal source/support files still match
the frozen pre-existing worktree snapshot.

## Trigonometric donor correctness controls

Full independent reads of `sin_cos.c`, `asin.c`, and `atan.c` show the tradeoff
between named functions and field-friendly complex exponential/logarithmic
representations. Calcium's default often introduces complex intermediates even
for real inputs. Hyper already has finite-real inverse-trig domain rules,
reflection and deferred kernels; no replacement by these complex formulas is
justified. The current random sin/cos test permits Unknown comparisons and does
not exercise output-state tangent behavior. Existing Machin tests use a separate
real-only helper, not the production complex atan-logarithm branch selector.

`flint-trig-state-probe.c` reproduces two independent current-donor problems:

1. Tangent's special handler asks whether **res**, rather than **x**, is Unknown.
   Across tan, tan_direct, tan_exponential, tan_sine_cosine and cot, 25 of 175
   controls fail. An Unknown input becomes Undefined when the output initially
   holds zero, Undefined or i; Undefined/unsigned-infinity inputs remain Unknown
   if the destination was Unknown. The other 150 special controls pass, including
   signed imaginary infinities and in-place cases.
2. `atan_logarithm` overwrites res with 1-i*x before evaluating x's imaginary
   enclosure for branch selection. When res aliases x, `atan_logarithm(2i)`
   disagrees with the known principal value pi/2+i*ln(3)/2. Direct and separate
   output forms agree with it, and all three -2i controls agree. The source's
   mistaken branch gives -pi/2+i*ln(3)/2. The entry-point documentation does not
   state a non-alias restriction; this observation is not a blanket guarantee of
   alias support for every donor API.

The native probe exits 1 for 26 mathematical failures among 181 controls. Memcheck
reproduces identical rows, reports zero errors and frees every heap block. The
archived implementation contains the same code mechanisms, but only current FLINT
was executed. No donor fix or external issue was submitted. These failures do not
imply corresponding Hyper bugs: Hyper's owned finite-real constructors have no
mutable destination alias and no complex atan principal-branch selection.

## Storage and evidence continuity

Temporary quota failures initially blocked builds and then patch application;
neither failed patch landed. The audit-owned 144 MB `/tmp/calcium-prototype-target`
was moved without deletion to workspace `.audit-builds/calcium-prototype-tmp-preserved`.
The user then freed additional temporary space. Builds reused the existing two
targets with incremental compilation disabled; no unrelated source or artifacts
were removed. An erroneous benchmark-only `offset` method call was corrected to
public multiplication; failed build logs remain available.

`verify-checkpoint.mjs` verifies the frozen source footprint, source-directory
coverage, corpus Cartesian membership, exact outcomes, test totals, benchmark
grouping/instrumentation and both successful/known-failing memory observations.
It does not certify that an unread file was read or that unmeasured consumers pass.

## Next work / retention gate

1. Resolve log-proof scheduling/reuse without repeatedly rebuilding failed proofs
   or letting a failed low-budget attempt become a permanent nonzero/zero claim.
   Preserve the full proof domain; do not hide regressions by dropping hard cases.
2. Preserve special-function relations without losing kernel-cache reuse or
   structural range facts after serialization. Qualify the general nested-add
   cancellation idea independently; the combined erf prototype is not selected.
   The root/exponential experiment likewise needs semantic relation retention
   without discarding hot operand caches; its measured fresh-path benefit remains
   a candidate, not a retained production optimization.
3. For a revised candidate, rerun independent controls, matched fresh/shared
   measurements, allocation/size and proportional downstream gates before retention.
4. Continue unread Calcium/FLINT files and all other pending ecosystem targets.
   Special-function relations and generic field/cache candidates remain open.

No production change has been made or retained in this checkpoint. No broad
performance gain, full Calcium audit completion or ecosystem completion is claimed.
