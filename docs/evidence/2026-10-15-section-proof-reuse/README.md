# Reuse an identical one-sided section proof

Baseline: `3a2d3fd1b`, with production code and all 35 pending native files
verified against the closed slice/projection qualification. That qualification
records the substantial middle-slice cost regression; no code has changed at
the start of this experiment. All earlier trials and artifacts remain preserved.

CSGRS owns the union of the filled sections approached from above and below.
If their ordered directed segment indices and closure flags are identical in
one native `MeshSectionZ`, both refer to exactly the same constructed points
and source records under the same policy. One winding proof can satisfy both
claims, and union idempotence permits reusing its result. No geometric tolerance,
scalar equality guess, or new fill algorithm is involved. Native intersection
classification and coplanar admission still execute. Different section records
must retain the existing computation.

This is a proposed performance-only change under P4, P17–P20, P25, P27, P35–P39.
It must preserve all 25 semantic regressions, qualified certainty, native
source records, original full-resolution sphere/frustum workloads, and exact
output rows/checksums/materialized output sizes against the qualified baseline.
Tests establish set meaning separately from deterministic serialization. Cost
comparison must include completed construction, operation, validation,
materialization and drop with declared cache states, raw samples, allocation
counts and binary sizes. An output/provenance regression disqualifies the
shortcut; no policy or fixture may be weakened to retain it.

Disk space is limited. Compiler consolidation v1 only hard-links identical
non-executable compiler files in closed prior trials. All paths, bytes, unique
files, executables and evidence stay available. New mutable build targets must
use independent inodes. No prior target is resumed. Resource limits remain
external test containment, not internal work/byte/cancellation guarantees.

## Acceptance decision

**Kept**, with the scope and trade-offs in [decision-v1.json](decision-v1.json).
Only `src/solid.rs` changes production behavior. Native section classification
still runs before the shortcut; different above/below records and coplanar
admission retain their existing computation. Hypercurve already implements
union idempotence; CSGRS now avoids constructing and proving its identical
second operand. The input-identity test compares indices within one owned
section, not coordinate equality or a stale cross-call cache key.

All 25 existing regressions pass before and after. They independently check
exact set area and membership, signed multiplicity, determinant volume,
manifoldness, typed errors, policy receipts and cache/mutation sequences.
[The 43-case comparison](candidate-v1/comparison.json) preserves ordered exact
boundary/profile rows, roles, area expressions, native carrier availability,
curve families, scalar parameter domains, public arrangement-source flags and
operation certainty byte for byte. It includes the original 528-triangle
sphere and 12-segment README frustum. This comparison is additional evidence,
not an independent oracle or certification of analytic fidelity.

[Cargo qualification](cargo-v1/result.json) passes workspace formatting,
curve-only and minimal all-target Clippy with `-D warnings`, all 231 curve-only
library/integration tests, and all 96 minimal tests. These configuration totals
overlap. No test, policy, fixture resolution or native pending change was
altered. No binding exposes this operation. The full CI matrix, release/user
path, inherited provenance and internal resource bounds remain open.

## Costs and unfavorable results

[All raw commands and samples](cost-v1/protocol.json) and
[the assessment](assessment-v1.json) retain 72 fresh-process samples: four
completed cube operations, three cache conditions, three repetitions and two
revisions. Each sample warms up 128 operations and times 128 more. Input
construction, exact independent square checks, boundary materialization,
expression JSON, checksum and drop are included. All 9,216 timed operations
complete strictly; exact rows, output sizes and checksums match across
revisions for all 512 dumped input variants. Native/scalar caches are not
globally cold when modeling caches are disabled.

| Operation | Warm median | Varied median | Disabled modeling-cache median |
| --- | ---: | ---: | ---: |
| Middle section | −2.3% | −42.1% | −40.4% |
| Bottom section | −2.2% | −1.1% | +2.8% |
| Top section | −2.5% | +3.9% | +2.1% |
| Projection | −3.3% | −2.0% | −2.7% |

Middle-section varied/disabled allocations fall from 3,300 to 1,862 per
operation, requested bytes from 370,358 to 208,154, and peak live bytes above
the warmup state by 3,504. Retained bytes after 128 operations increase by
1,376. Other cases have identical allocation counts and requested bytes.
The standalone cost executable shrinks 2,880 bytes, while the CSGRS rlib grows
15,304 bytes; neither is a whole-distribution size measurement.

The original full snapshot corpus also completes three times per revision,
with identical outputs. Its process wall samples are approximately 0.067
seconds for both versions, with scheduler/subprocess overhead; RSS ranges
11,372–11,604 KiB before and 11,388–11,752 KiB after. This does not establish a
corpus-wide speedup. The host is shared and unpinned, only three samples are
collected, and allocation instrumentation affects timing. Adverse controls
and retained-memory costs are part of the decision, not discarded as noise.

## Reproduction and preserved trials

`baseline-v1` passes all 25 regressions but its snapshot cannot compile against
serialization-disabled native libraries. Its commands and errors remain
preserved. `baseline-v2` and `candidate-v1` use the same direct-rustc curve-only
build command and the exact same serialization-enabled native libraries from
the previous qualified all-feature build. The
[actual compiled dependency metadata](build-identity-v1/result.json) matches,
including encoded crate hashes; all native inputs are hash-guarded. Separate
Cargo validation covers curve-only/minimal feature isolation. The earlier E4
cost build used another configuration; its timing is not a matched baseline
for this experiment.

Each phase records complete source hashes, patches, commands, logs, resource
measurements and source guards. Scripts and closed target directories are
immutable. Reproduction requires a fresh phase/target and the recorded toolchain,
dependency artifacts and inputs; do not rerun a command into a closed target.
Compiler consolidations v1–v3 link only byte-identical non-executable files in
closed trials, keeping all paths, unique artifacts and executables. V2's
lexical ordering selected the newer copy, reclaimed no bytes and split old
inode relationships without changing content; v3 reconciles and verifies all
old link manifests against the most shared existing copies. The final
[closure](closure-v1.json) verifies prior evidence, all 35 pending native files,
and that only `current_work` changes in the properties ledger. The complete
properties goal remains active.
