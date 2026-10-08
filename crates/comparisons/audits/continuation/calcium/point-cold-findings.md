# Checkpoint 58 — cold-first-query and repeated-state qualification

The unchanged isolated demand-gated repair passes the cold native/WASM semantic
gate. Every demand result matches eager recovery, including the first query in
each fresh process/instance. There is no new production transfer or donor read
coverage. This is progress on the full audit, not its completion.

## Scope and source binding

- Retained baseline: `e-plan-qualified-candidate`; eager repair:
  `point-image-qualified-candidate`; demand repair: the existing 175-file
  `point-demand-candidate/hypersolve` with unchanged retained dependencies.
- The checkpoint 57 source binding remains
  `d5472beb99abd3f0526d978f4e27fb0446a1ac425dfd52fe3265bdccca4ba80f`.
  All 956 live source identities and all compared snapshots are rechecked.
- No algorithm is revised. The shared `point-history-base.rs` remains the exact
  qualified checkpoint 56 prefix. New harnesses serialize complete inputs and
  reports after each of nine explicitly observed queries.
- Source coverage remains 1,415 complete files, 20 partial files and 183,653
  uniquely read donor lines for the Calcium/FLINT continuation only. These
  counts do not represent completion of the original reference inventory.

## State protocol

For every baseline/eager/demand and native/WASM combination, execute all 48
authored cases, two policies, four initial histories and three lifecycles:
1,152 groups and 10,368 fully recorded queries per combination. The entire
campaign has 6,912 groups and 62,208 query records, plus one terminal record per
group. It creates 3,456 native child processes and 3,456 fresh WASM instances.

Each group starts in a fresh native process or import-free WASM instance. There
are **no unrecorded query warmups**. Constructors and selected initial history
(constructed, -64-bit refinement, -4096-bit refinement or JSON round-trip) are
explicit setup and may initialize caches. This is not an absolute zero-cache
claim. Calls 1–8 are intentionally later calls in that same process/instance.

- Retained: use the same input roots for all nine calls.
- Fresh: drop prior input graphs before rebuilding and applying the selected
  initial history for each call. Process-level constants may still be reused.
- Round-trip: prepare the selected initial history for call 0, then serialize
  and deserialize every scalar input field between subsequent calls. The
  recorded `history` names the initial history, not the current cache contents.

Each returned report is fully serialized and dropped before the next query;
only output bytes are retained. Inputs' serialized records are asserted
unchanged across each query. These assertions do not imply caches are untouched.
The native driver records each process's identity, arguments, terminal status,
time boundaries, raw length and hash. The WASM driver records corresponding
group hashes, arguments and linear-memory sizes for each fresh instance. All
group records and termination markers are checked independently afterward.

WASM uses Node v22.22.2 / V8 12.4.254.21-node.39, one compiled module per variant,
with no imports and a new instance/linear memory per group. Explicit host GC
every eight completed groups limits accumulated unused instances; the driver
does not retain old instances. This is not a separate Node process per group.

## Independent mathematical and comparison results

The exact field/identity oracle from checkpoint 55 evaluates every captured
query, without using Hyper comparisons, donor arithmetic or floating overlap.
It checks authored inputs, exact carrier polynomials up to nonzero rational
scale with multiplicity preserved, selected values, root counts, witnesses and
interval images. Unsupported computational nodes fail closed. Deliberately
changed witness, polynomial and endpoint controls are rejected.

All **1,751,136 independent checks pass**, with zero reported mathematical
failures. These are repeated observations of the authored corpus, not 62,208
independent mathematical problems or a universal proof over all expressions.

The native and WASM raw query/terminal files are byte-identical for each variant.
The following totals apply separately to either platform:

| Result | Baseline | Eager repair | Demand repair |
| --- | ---: | ---: | ---: |
| Observed queries | 10,368 | 10,368 | 10,368 |
| Transformed | 5,044 | 8,500 | 8,500 |
| Returned exact witnesses | 3,316 | 6,772 | 6,772 |
| Invalid transformed evidence | 3,456 | 0 | 0 |
| Denominator may contain zero | 648 | 648 | 648 |
| Undecided | 178 | 178 | 178 |
| Non-isolating image | 178 | 178 | 178 |
| Invalid input evidence | 864 | 864 | 864 |
| Independent checks | 268,816 | 303,376 | 303,376 |

Demand and eager have **10,368 identical complete query records per platform**,
not merely equal status codes. Versus baseline, both recover 3,456 records and
leave 6,912 unchanged. All gains return exact witnesses, with identical input
records. There are no lost answers, lost witnesses or other differences.

Among the 1,152 first queries per variant/platform, baseline transforms 564
with 372 witnesses; either repair transforms 948 with 756 witnesses. Thus 384
first-query records improve and the other 768 agree. Those 384 groups are the
same 128 case/policy/history combinations repeated over three lifecycles, not
384 new independent defects. Initial results agree across all three lifecycles.

## Serialization changes bounded decision availability, not exact values

Four groups per variant/platform change status after call 0:
`36:0:2:roundtrip`, `36:1:2:roundtrip`, `37:0:2:roundtrip`, and
`37:1:2:roundtrip` (case, policy, initial history, lifecycle). These are Add and
Subtract for a zero-root interval with endpoints `z ± 2^-3000`, where
`z = sin(e)^2 + cos(e)^2 - 1` is exactly zero but not symbolically resolved by
the bounded query. The second input is the exact zero point.

After deep initial refinement, call 0 is Transformed with an exact witness.
After serialization, calls 1–8 are Undecided under STRICT and
NonIsolatingImageInterval under APPROXIMATE_512. All three variants behave
identically on both platforms. Every serialized input remains identical and
the independent oracle proves the exact values unchanged. These 24 repeated
group transitions are not 24 independent defects or candidate regressions.
No other full record changes within any group; retained/fresh sequences are
stable over these nine calls.

This is consistent with the current scalar architecture: computable node facts
and approximation caches are skipped/defaulted during serialization
(`hyperreal/src/computable/node/representation.rs:19–24`), as is Real's
primitive approximation cache (`src/real/arithmetic/representation.rs:54–55`).
`src/serde.rs:8–18` uses the derived serializer/deserializer. Serialization
omits these accelerators; it does not mutate the original object's cache.
Exact value persistence is not a promise that a resource-bounded predicate
will retain its previous ability to decide. Do not add trusted serialized
cache facts or weaken strict proof obligations to conceal that distinction.

## Builds, failures and resources

The final release native/WASM harness builds and native all-target / WASM library
Clippy gates pass warning-free for all three variants. Rustfmt check passes.
These are harness gates, not new runs of the full solver or consumer suites;
checkpoint 57's qualified solver tests and memory results remain their own gates.

The original native build succeeded, but Clippy rejected an unnecessary unwrap
in the harness's Option control flow (Cargo exit 101). The first explicit resume
then incorrectly expected wrapper exit 1 rather than Cargo exit 101 and stopped
before building. Both failures and the outer build failure remain captured.
`point-cold-clippy-fix.patch` preserves the exact control-flow edit, and the
resume failure trace preserves the exit-code assertion correction. Final
qualified builds use the corrected source. No observations use the initial
unqualified binary, and no oracle or recorded observation was changed.

- Successful resumed build: 2026-09-10T23:08:40.399Z–23:10:02.953Z, exit 0.
- Collection: 23:10:40.575Z–23:11:18.240Z, exit 0; all six collectors exit 0.
- Mathematical check: 23:12:48.423Z–23:13:17.230Z, exit 0, empty stderr.
- Six frozen binaries in `/tmp/calcium-point-cold.3RcRIa`: **14,358,510 bytes**.
  Existing source trees and the shared Cargo cache are reused; no source copy,
  cleanup or deletion occurs.
- Raw query output: **199,526,880 workspace bytes**; group metadata:
  **1,941,696 bytes**; total **201,468,576 bytes**, excluding check/build logs.
- Capacity capture: **16,247,853,056 bytes available in /tmp**.
- WASM linear memory after collection spans 1,310,720–1,638,400 bytes. These
  values include harness/output state and are not per-query peak demand, RSS,
  allocator costs, lifetime bounds or CPU measurements.

## Decision and next work

Close the cold-first-query and extended native/WASM semantic gates for this
unchanged demand candidate. No further algorithm revision has been selected.
The earlier matched native costs still stand: improved nonpoint controls,
unchanged same-result allocation/peak counts versus baseline, slower timing
controls and extra rejected-report work on newly recovered point queries.

Before any retention decision, run matched extended WASM costs and final
consumer/representative-size qualification for the selected version. This
checkpoint does not infer speedups from collector wall time, claim memory
cleanliness without a new memory run, or transfer old eager consumer/size results
to the demand variant. The separate power-sum candidate, remaining donor/source
reads, all other original references and complete inventory reconciliation stay
open. Five retained continuation transfers remain unchanged.

Evidence is bound by `point-cold-manifest.json`; run `node verify-point-cold.mjs`.
The verifier rechecks checkpoints 48–57 and their preserved failures, not the
full historical checkpoint 47 chain; its latest complete capture remains
2026-09-10T04:56:34.285Z. Root report and progress notes are intentionally mutable
summaries, separate from the bound checkpoint artifacts.
