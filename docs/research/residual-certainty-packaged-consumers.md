# Residual-certainty packaged consumers

The isolated repair `aad7409a5928e309812d9a5cc361d46313685266` now has finite
qualification through verified package archives. The owner is Hypersolve's
certification and diagnostic layer, under P6/P8/P9/P19 and interval-plan phases
7 and 14. The missing witness was use of the corrected residual-certainty
contract through shipped sources without workspace fallback or stale registry
source. This does not adopt the repair or close its remaining resource gates.
The [certainty review](residual-certification-ownership-review.md) retains the
original failing witnesses and memory results; the
[portable review](failed-constraint-ownership-review.md#native-and-webassembly-report-consumers)
retains earlier workspace-consumer measurements.

## Verified distribution boundary

The fixed program revision `c9f1f9ac55216b762e9eb97672de1e66f3b8a3fb`, measured
current revision `4192d8c726d3333a31cb7eddc30cc0d464d61411`, and isolated candidate
are each packaged with Cargo 1.95.0, locked offline resolution and normal package
verification. All six required crates are included: Hyperreal, Hyperinterval,
Hyperlattice-interval, Hyperlattice, Hyperlimit and Hypersolve. Eighteen archives
pass VCS/clean-state validation and 1,344 included-source comparisons
against Git objects. Generated manifests, locks and VCS records are retained.
The two affected primary source files remain identical to the measured current
revision; later documentation commits are not relabeled as measured builds.

Each revision uses a fresh Cargo home sharing only public registry caches.
The unchanged release-cache test demonstrates its stale-source negative control
and fresh-home positive control. Consumer metadata is checked in twelve
feature/target configurations. Its four active Hyper libraries resolve only
from unpacked archives with exact versions; the two optional interval libraries
are separately package-verified. No workspace library supplies a fallback.

The first attempt produced unused optional-patch warnings. It was stopped under
controlled interruption after six verified archives and 688 passing native
controls. The corrected harness patches only the four active dependencies,
uses a new target/artifact directory, and charges the original elapsed time to
the same campaign budget. All six repeated fixed archives are byte-identical.
Both attempts and the original warning remain preserved.

## Completed contracts and final images

The fixture, operation, validation and entry-point sources are unchanged copies
of the earlier portable corpus: 113 cases and 196 policy configurations, reused
or rebuilt inputs, cold/eight-call warmup, and absent/passive/active tracing.
Added corrected outcomes run only on the candidate. Twelve final native/WASM
images pass 12,960 controls, all compared byte-for-byte to the corresponding
workspace report and trace. There are 10,608 additional full comparisons
across package boundaries/targets and 4,248 nonempty active trace rows.
Independent exact fixture assertions precede report formatting. This is finite
report consumption evidence, not an exact native-replay codec or universal
cross-platform equivalence proof.

Release settings remain opt-level 3, thin LTO and one codegen unit. Every image
is compared against package images built at the same checkout and unpack paths.
All original prospective absolute/relative caps and fixture reserves pass.
File and native allocated-section growth is capped at 8 KiB and 0.5%
incrementally, 16 KiB and 1% cumulatively. Code growth is capped at 4 KiB and
0.5% incrementally, 8 KiB and 1% cumulatively. Both absolute and relative caps
apply, with current and fixed denominators respectively.
Each positive charge is retained separately; later savings do not erase an
inherited charge, and targets do not exchange allowance.

| Final image | Tracing | Candidate file bytes | Cumulative file / executable-code charge (bytes) |
| --- | --- | ---: | ---: |
| Native | absent | 2,750,272 | 24 / 0 |
| WASM | absent | 1,728,907 | 358 / 310 |
| Native | compiled | 2,906,912 | 0 / 0 |
| WASM | compiled | 1,886,760 | 358 / 310 |

Native allocated-section, text and named-code accounts are also retained in
the evidence. Native capacity/reserve is 16 MiB/2 MiB; WASM is 8 MiB/1 MiB.
These are qualification-fixture capacities, not application or firmware
partitions. WASM modules have zero imports and explicitly release their report
buffer. The maximum observed linear-memory size is
1,441,792 bytes, within the declared 256 MiB capacity/64 MiB
reserve. Linear-memory pages are not live-heap or stack measurements.

Strict Clippy passes for both features on native and WASM; formatting passes.
There are no consumer compiler or Clippy warnings. Cargo's package notices
about tests/examples/benches excluded by the existing manifests remain recorded
(892 notices across the three revision builds); package verification is not
reported as warning-free. No production manifest or test is weakened.

## Admission status

The isolated candidate branch is restored and the existing quiet watcher is
resumed without resetting its horizon or changing the frozen command plan.
External routing work overlaps this campaign; no runtime claim comes from its
build or functional durations. The two earlier DRC failures remain recorded.
This six-crate package check is not the full coordinated stack release.
Including the interrupted attempt, the campaign completed in 2,545.33 seconds,
below its 21,600-second cap. The driver checks total time before subprocesses;
it does not propagate a strict whole-campaign deadline into them. The completed
duration is checked independently, without claiming solver-internal work or
cancellation bounds.

Original native memory failures and positive charges remain open. Owning-crate
final artifacts, complete consumer capacity, cancellation/retry, runtime and
composition with the four earlier repairs still require qualification. A source
review confirms that batch/removal searches rerun scalar certification instead
of accepting residual-ball reports; it supplies no new runtime or general
closure claim. No production source, inventory row, accepted size account or
memory account changes. All 57 accepted size accounts and zero accepted memory
accounts remain intact; the broader fifteen-phase plan stays active.

Frozen evidence: `docs/evidence/2026-11-17-interval-residual-certainty-packaged`, local commit
`c76d80440c31d6c1d59dff216a689d27e036fe79`; 253 files / 56,808,003 bytes, SHA-256
`295c4976b18427b073f9990d010e4510f7b333d4f584c746b6106ae6bbfd2407`. All earlier archive paths and index rows remain unchanged.
Both package attempts and the review snapshot are frozen. No push.
