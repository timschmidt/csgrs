The measured direct-projection version is rejected. All 42 scoped correctness
commands pass, but nine phases are slower than the unchanged baseline in every
fixed pair, retained memory rises in twenty-one phases, and every probe grows.
The [decision](decision-v2.json) preserves all favorable and adverse results.
Neither experimental source advances to native validation or main runtime.

This experiment compares three fresh sources: the unchanged endpoint/P4
baseline, the previously rejected recursive-view control, and that control
with direct rational, pi and quadratic projection. The projected variant
borrows coefficient slices through a private read-only trait and clones only
the exact rational components returned to callers. Compacting still
materializes an owned polynomial before reconstruction.

The [scope](scope-v2.json) binds the source review and fixed cost protocol.
Rational zero, pi-power matching, quadratic shape checks, opaque rejection,
sign publication and successful dispatch tracing keep their original meaning.
The existing owned/retained enum, graph-free closed payload, recursive
arithmetic, memo insertion and opaque ownership remain. The original retry
follows a failed reused-certificate pass after releasing its memo. No
recognition pass, cache fact, approximation, dependency, public API or budget
is added. P4 permits stronger cached facts, never contradictions or weaker
available facts.

Six new shared tests cover public projection queries, shape boundaries,
cancellation, high-precision sign retention and concurrent graph release.
Fourteen historical storage/memo/alias tests remain unchanged. Both
experimental variants retain five private view tests; projected adds three
for owned/retained shapes, opaque ownership, immutable buffers and payload
release. The source guards recover the original control's tokens after
removing only the declared projection changes.

The [qualification assessment](correctness-v2.json) independently verifies all
42 commands and their logs. Baseline passes 894 all-feature and 807 default
library tests, recursive control 899 and 812, and projected candidate 902 and
815. Each passes 28 doctests, the inherited cache/P4 checks, formatting, strict
default/all-feature/all-target Clippy and warning-denied docs. Focused and
configuration counts overlap; these are not unique or full-workflow totals.

All eight existing type layouts match, including 56-byte scratch coefficients,
32-byte closed coefficients and 64-byte cache values. Both experimental result
enums are 24 bytes. Memo inventories remain 1/1/2 for retained/original/opaque
controls; ordinary deep-copy sharing remains 0/0. These layouts do not measure
heap headers or total memory.

The fixed cost trial completed eighteen preflights and ninety measurements
across three unchanged suites. All builds preceded probes; timing and
allocation used separate executables. Every mathematical witness and
representation inventory matches. The [cost assessment](cost-assessment-v2.json)
verifies all 129 commands, raw logs, binaries and summaries. All three contrasts
share one new closed sample set and are not independent experiments. Earlier
samples remain closed; none is appended, pooled or discarded.

| Comparison | Slower in all 5 pairs | Slower in at least 4 | Faster in all 5 | Faster in at least 4 |
| --- | ---: | ---: | ---: | ---: |
| Recursive vs baseline | 1 | 28 | 7 | 13 |
| Projected vs baseline | 9 | 29 | 4 | 10 |
| Projected vs recursive | 10 | 12 | 11 | 30 |

Each contrast contains 68 timing and 68 allocation phases. These directions,
ratios and ranges are descriptive, with no significance or application-speedup
claim. The fresh recursive-control timing directions differ from its earlier
separate run; those runs are not matched comparisons. Large variation remains
visible in this sample set.

Projected conjugate-product first/warm/cloned queries are slower than baseline
in all five pairs, with median paired ratios 1.061595/1.059850/1.058736. Their
paired ranges are 1.043886..1.075333, 1.038217..1.121170 and
1.030894..1.117077. Integer canonical-boundary warm, signed-word
first/warm/cloned and wide-integer first/cloned queries account for the other
six all-five adverse phases. Wide-integer first/cloned median paired ratios
are 1.343974/1.183993, with substantial sample variation.

Against baseline, both experimental variants have the same requested-memory
deltas in every phase. Calls decrease in nine phases and increase in none;
allocated bytes decrease in nine and increase in fifteen; retained bytes
increase in twenty-one and decrease in none; peak bytes increase in twenty-one
and decrease in one by 96 bytes. Every paired delta repeats five times. The
largest increase is 16,000 bytes per 1,000 inputs.

Against recursive control, projection changes none of the four measured heap
counters in any phase or block. That establishes no additional measured heap
saving on these probes, not that projection can never avoid copying. Requested
heap counters exclude native C heaps, stack frames and allocator bookkeeping;
retention is phase-relative and does not measure eventual reclamation.

Favorable results remain in the decision. Projected multiradical-square
first/warm/cloned queries are faster than baseline in every pair, with median
paired ratios 0.903063/0.913401/0.924875 and 2,000 fewer allocations per 1,000
inputs. First/cloned still retain 16,000 additional bytes. Mixed-field-decline
cloned is the fourth all-five faster phase. Against recursive control, eleven
quadratic phases are faster in all pairs, while ten integer phases are slower
in all pairs. This trial does not establish the cause of the integer differences.

Projected probes grow by 5,072 to 5,080 bytes against baseline and by 856 to 888
bytes against recursive control. Production CLI size is unmeasured. No native
qualification, native costs, original fixture gates or full-report replay ran
for this rejected source. The parent still fails the 120-second board
worker-CPU assertion at 133.31248412 seconds and passes the separate 60-second
neck check-body assertion. Fixtures and limits remain unchanged.

The [version 1 failure assessment](preparation-failure-assessment-v1.json)
preserves a preparation failure before compilation: rustfmt removed an optional
trailing comma and collapsed an equivalent projection closure. Version 2
corrected those expected fragments and reproduced every formatted version 1
source byte. The original inputs, failure and generated sources remain frozen;
version 1 ran no qualification or costs.

The next review considers a scoped read callback for a closed right product
operand while retaining the parent's Box-backed cache and owned return type.
It must resolve lock reentrancy, writer blocking, callback lifetime and exact
work/owner accounting before any candidate is prepared. Avoiding an Arc header
and a recursive result enum is a source-based hypothesis, not an established
improvement. Earlier root retained-copy coverage of 0.6244 percent is not a cost
estimate. Both original native gates and the full properties plan remain open.
