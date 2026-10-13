The endpoint trial's nine broad controls that were slower in all five original
pairs make no endpoint calls in their measured loops. Their recorded call,
allocation, deallocation and cache-event counts agree across both trees. This
rules out additional recorded work in those loops; it does not explain away
the original timing results or establish production retention.

[Interpretation](interpretation-v1.json) and
[checked observations](assessment-v1.json) retain the complete results. The
original [fixed performance comparison](../2026-10-12-endpoint-coordinate-reuse/controls-assessment-v1.json)
remains unchanged. No instrumented duration is an additional timing sample.

The five arc phases with increased memory counters are reproduced exactly:

| Phase | Extra allocation calls | Extra requested bytes | Extra live requested bytes at phase end |
| --- | ---: | ---: | ---: |
| Opaque construction | 11 | 256 | 88 |
| Off-circle warm | -1,573 | -43,472 | 25,168 |
| Aborted-unknown first | 128 | 3,080 | 120 |
| Approximate non-endpoint first | 3 | 168 | 168 |
| Near-endpoint off-circle warm | -19,572 | -718,888 | 46,904 |

Both warm phases add 143 allocations and 25,168 requested bytes inside rational
linear-retention scopes. Counting construction, first/warm/cloned queries and
the three intervening windows, each family's live requested-byte increment is
125,664 bytes smaller in the candidate. That calculation uses the root probe
totals, where each allocation is counted once. It preserves the adverse phase
rows and does not prove reclamation after every owner is dropped.

Opaque construction makes no endpoint call and creates one additional 80-byte
evaluation-cache allocation. The aborted-unknown first-query increase occurs
inside policy equality and zero testing, also with one additional 80-byte
evaluation-cache allocation. The approximate non-endpoint increase occurs
inside squared distance with unchanged call counts; these counters do not
identify its precise retained payload. The respective 88-, 120- and 168-byte
increases persist through those families' cloned measured queries.

The broad warm-area preparation window saves 96 allocations and 3,840 requested
bytes in endpoint classification. Its later measured loop allocates nothing
and makes none of the instrumented calls. Mixed-quadratic aggregation and unit
rectangle construction have identical counters in all four recorded windows.
All nine selected broad controls also agree in setup and postcondition checks.
Earlier state, code generation/layout and measurement variability remain
possible explanations for the slower original timings; none is established
as their cause.

Two calibration tests and five existing point-equality tests pass in each
diagnostic tree; the candidate's five existing endpoint regressions also pass.
Both trees pass formatting and strict release, all-feature, all-target Clippy
for hyperreal, hypercurve and hyperdrc. Four fresh probe processes preserve all
original witnesses, including 231 broad operation rows per variant. Every arc
allocation row matches every one of the five original memory pairs. Separate
Cargo targets and recorded manifests, features and hashes bind the executables
to their intended sources.

The diagnostic adds integer-only thread-local scopes, without retaining scalar
or geometry owners. The 97 arc and 905 broad windows per variant are retained
in [the full comparison](all-window-comparisons-v1.json.gz). Scope rows overlap,
exclude native C heaps, and are not propagated to worker threads. Node creation
counts exclude the surrounding Arc allocation. Instrumentation overhead and
equal counters cannot establish equal runtime or universal cache completeness.

The source remains available only for continued isolated investigation. The
original full-board worker limit still fails at 138.312559051 seconds against
120; the separate copper-neck check passes its unchanged 60-second assertion.
Next is a fresh profile of the uninstrumented candidate on the complete boards.
Production admission, wider qualification and the full properties plan remain
open.
