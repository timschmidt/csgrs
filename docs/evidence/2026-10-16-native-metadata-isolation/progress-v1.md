# Isolated metadata-order validation

Do not admit this runtime candidate or the 35 pending native files on this
evidence. All non-runtime qualification commands pass, and both complete board
reports remain identical. The original full-board and copper-neck runtime gates
still fail. Warm duplicate rejection is slower in all five paired samples.

The [contract](contract-v1.md), [decision](decision-v1.json),
[qualification](qualification-assessment-v1.json), [cost assessment](cost-assessment-v1.json)
and [report comparison](report-replay-v1/result.json) retain their separate claims.

## Scope and original workload gates

The target revision is `73472622dd4034b8b8777cb2a116c4020f6bb328`.
The live production and test sources, including all 35 pending native paths,
remain unchanged. Fresh baseline and candidate copies contain identical three
acceptance tests. Their only production difference is the order of the existing
duplicate-evidence conjunction in Hypercurve: integer arrangement-source records
before geometric equality. No earlier rejected scalar or endpoint change is included.

| Original gate | Observed assertion | Unchanged limit | Outcome |
| --- | --- | --- | --- |
| Full board | 176.419625977 worker CPU seconds | 120 seconds | Failed |
| Copper neck | 86.132483507 check-body wall seconds | 60 seconds | Failed |
| Gerber | Original smoke assertions | Unchanged | Passed |

Both full board archives ran. Finding counts were 12 and 197; those counts alone
are not a set-equality oracle. Whole-command times include compilation or setup
and must not replace the original assertion metrics. These standalone runs do
not establish an application speedup against the earlier baseline.

## Qualification and semantics

The baseline phase passed all 6 commands. The candidate phase passed 14 of 16;
only the two original runtime gates failed. Format checks, all-feature/all-target
Clippy for Hypercurve and Hyperdrc with `-D warnings`, documentation with warnings
denied, the CLI build, and source/build-input integrity checks passed.

| Test command | Passed | Failed | Existing ignored |
| --- | ---: | ---: | ---: |
| Hypercurve, no default features | 2463 | 0 | 28 |
| Hypercurve, all features | 2500 | 0 | 33 |
| Hyperdrc, all features, original three gates run separately | 1643 | 0 | 0 |
| CSGRS, all features | 405 | 0 | 1 |
| CSGRS, no default features | 132 | 0 | 0 |

Counts include the test/doctest summaries in each command. Feature configurations
and focused subsets overlap; do not sum them as distinct tests. No existing
ignored test was enabled, removed or newly ignored by the candidate.

The three shared regressions cover independently constructed equal coordinates,
absent/equal/unequal provenance under both policies, duplicate-before-source-reuse
error priority, preservation of available scalar proofs, and later refinement.
They pass in both qualification copies and both instrumented probe copies.
These finite cases do not establish complete symbolic point equality: the existing
`Point2` contract still allows `false` when equality cannot be proved.

## Fixed local costs

Both probe builds passed strict Clippy and the same three regressions. The fixed
schedule completed 20 fresh processes and all 360 rows: five alternating pairs
per mode, six metadata cases, and three validation phases. Timing uses 1,024
inputs per case; allocation measurements use 128. The duplicate case is an
expected rejection control, not successful modeling completion.

Each cell below is the median paired candidate/baseline time ratio, followed by
how many of five pairs favored the candidate. Every control is shown.

| Case | First validation | Warm validation | Cloned validation |
| --- | ---: | ---: | ---: |
| unequal_sources | 0.517167 (5/5) | 0.493041 (5/5) | 0.189301 (5/5) |
| absent_sources | 0.976125 (4/5) | 0.967117 (3/5) | 0.979780 (3/5) |
| mixed_sources | 0.469487 (5/5) | 0.542544 (5/5) | 0.401282 (5/5) |
| equal_sources | 0.982327 (3/5) | 0.983075 (3/5) | 0.975057 (3/5) |
| duplicate | 0.992335 (3/5) | 1.069022 (0/5) | 1.022445 (1/5) |
| singleton | 0.993157 (3/5) | 1.008383 (2/5) | 0.986701 (4/5) |

For each 128-input allocation sample, unequal-source cases save 8,832 allocation
calls and 353,280 requested bytes; mixed-source cases save 4,608 calls and 184,320
bytes. These deltas repeat in all three phases and five pairs. No measured
allocation metric increases. Retained and peak-extra Rust allocator deltas do
not establish process RSS, native allocator costs or a global heap bound.

Both variants use the same TLS allocator wrapper; counting is enabled only in
allocation mode. Fixture construction, between-phase input cloning and output
buffer reservation are outside the timer. Per-request cloning and returned
region materialization are inside it, with outputs retained until the phase ends.
Local timings are small authored controls, not a representative workload corpus.

Both production CLI executables are 42,996,752 bytes. Both probe executables are
41,500,920 bytes; test/probe size is distinct from production CLI size.

## Complete reports and retained evidence

The four fresh CLI processes use common extracted input paths and both original
archives. Exit code 1 is the expected CLI status for a complete report containing
active findings; it is checked separately from report completeness.

| Fixture | Report bytes per variant | Findings | Full bytes and parsed values |
| --- | ---: | ---: | --- |
| cparti-fpga | 147297 | 12 | Identical |
| hvp109a | 128420 | 197 | Identical |

No report field, path or whitespace was normalized. Raw reports, compressed
copies, command output, source and binary identities, archive/entry identities,
compiler/hardware details, all sample rows, and resource observations are retained.
Complete report equality supplements independent authored semantic tests; it
does not supply an independent geometry oracle on its own.

The [checkpoint](checkpoint-v1.json) records the final artifact audit and its
inherited missing-path expectations. The parent has 27,093 missing paths; this
work does not claim historical completeness. Only the raw `current_work` JSON
value may change in the execution ledger; other bytes and decoded keys remain
bound to the required `9025de62` reference.

## Remaining work

External 1,800-second command and 8-GiB per-process address-space limits provide
containment, not internal total-work, cancellation or retained-heap contracts.
Inherited provenance, clean external package/MSRV/release checks, complete workflow
validation, formal verification, directed campaigns and instrumented coverage
remain open. No push or runtime admission is claimed.

Before more performance work, reproduce the format-boundary composition cases
in [the inspection note](followup-inspection-v1.md) in a fresh phase. They are
currently hypotheses from source inspection, not executed findings. The full
properties plan remains active and incomplete.
