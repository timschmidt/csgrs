The isolated DRC allocator candidate passes both original native gates and
preserves every complete report. It remains isolated pending review of its
inherited scalar-cache and curve changes. [Decision](decision-v1.json).

[Qualification](qualification-assessment-v3.json) verifies 34 distinct commands:
27 preserved successes/results and seven continuation commands. Both variants
pass formatting, default/all-feature strict Clippy, warning-denied docs, two
allocator controls and all 1,646 DRC tests. Focused controls overlap the suite;
independently linked integration/doctest drivers retain their own allocator.
Both CLI and library-test executables select the intended allocator. The candidate
uses pinned mimalloc with no extra features or C malloc override; reusable-library
consumers retain allocator choice, with a new direct native build dependency.

The baseline board assertion fails at 132.223696162 worker-CPU seconds / 120.
The candidate passes that original limit, and both variants pass the 60-second
copper-neck check-body limit. Passing assertions do not print their measured
internal durations; whole-process timings do not replace those assertions.

The [fixed application assessment](application-assessment-v2.json) covers four
preflights, 20 board measurements and 10 batches of 50 fresh help processes. All
five prescribed blocks are retained, with no extension or filtering. Medians:

| Fixture | CLI wall seconds, baseline → candidate | CLI CPU seconds, baseline → candidate | Peak RSS KiB, baseline → candidate |
| --- | ---: | ---: | ---: |
| CPArti | 107.462 → 85.166 | 106.740 → 84.600 | 1,891,904 → 1,773,996 |
| HVP109A | 25.652 → 21.728 | 25.470 → 21.580 | 532,516 → 524,528 |

Wall time, total CPU and peak RSS decrease in all five pairs for both fixtures.
Median CPU use is 20.7% and 15.3% lower; median RSS is 6.2% and 1.5% lower.
These are descriptive observations on this Linux x86-64 host, not significance
claims or universal performance guarantees. Fresh processes share OS file caches;
RSS includes allocator/process overhead and is not live/requested payload.

Help startup worsens in every pair: median batch-average wall time rises from
2.4075 to 2.9272 ms per process (+21.6%, about 0.520 ms), including identical
launch/wait overhead. Startup CPU also rises in every pair. The CLI grows by
158,256 bytes and the library-test executable by 21,608 bytes. These adverse
results remain part of the decision.

All 24 full reports match complete bytes and JSON values without normalization,
including the earlier endpoint/metadata reports: CPArti is 147,892 bytes with 12
findings; HVP109A is 128,807 bytes with 197 findings. All 500 help outputs match.

[V2 preparation](preparation-v2/protocol.json) verifies the 10,132-file baseline,
10,133-file candidate, identical controls and 150 dependency archive/source
inputs. [V1 preparation failure](preparation-failure-v1.json) records the absent
Cargo checksum manifest; V2 verifies the locked archives instead. The [V2 linkage
reader failure](qualification-failure-v2.json) is preserved. V3 verifies each GOT
dispatch through its exact ELF relocation to `mi_malloc_aligned`, reusing all 27
completed commands without rebuilding or rerunning passed tests. Old runners and
all executed source trees remain unchanged.

The [admission boundary inventory](admission-boundary-inventory-v1.json) identifies
12 modified guarded paths and five added source paths relative to the main tree,
including eight inherited kernel paths. It is an inventory, not credited as source
reading. The next step is to review those inherited changes and their preserved
adverse cost evidence, compare documentation and specify remaining combined
admission checks. This DRC allocator result does not erase library-consumer
tradeoffs. The 1,134-file main source guard and all 35 pending native paths remain
unchanged; the full properties/targets/future-work plan remains open.
