# Native memory and stack through replay and cleanup

The legacy adapter introduced a stack temporary for the successful return value.
On x86_64 glibc, the Hyperpath entry point used 448 bytes more touched stack
than the primary caller in each of 18 workloads. Direct Hyperdynamics calls
matched the primary caller. In the measured executable, the planner wrapper
reserved 432 bytes, saved a register, called the owner with an address in its
frame, then copied 424 bytes from that frame to the caller. The wrapper
preserves Rust ownership and avoids deep cloning, but moving a report still has
a physical cost.

Isolated commit `62bcec291` adds `#[inline]` to the two short forwarding
functions. Their computation and error mapping are identical to the preceding
candidate. All 18 legacy workloads then used 48 additional stack bytes. The
proposed host stack limits are 256 bytes **and** 2%, both incremental and
cumulative. The measured result leaves 208 bytes of absolute allowance, also
subject to the remaining percentage allowance. This proposal applies to the
instrumented native projection client with one worker.

The fixed program revision, current primary, previous legacy adapter, direct
owner, and revised legacy adapter ran the same rational workload. Each tested
1 span and 1 axis, 4 spans and 3 axes, and 32 spans and 8 axes under both
policies, with tracing absent, compiled but passive, and active. Four cycles in
each fresh worker constructed inputs, rejected empty input, planned, retained
and independently replayed all rows, materialized the active trace, and dropped
the reports and inputs. Fresh control processes checked complete reports and
ordered trace counts in addition to the independent exact limits and first-row
bottleneck assertions. Across five build variants, 180 worker histories cover 720 complete cycles. All 72 comparisons against the fixed control matched.

Peak requested and glibc-usable heap across the portfolio were 399,479 and
421,088 bytes. Each matched its primary value for all three consumer paths
in every domain. The largest post-cleanup deltas were 20,488 requested and
20,944 usable bytes. All observed histories were stable from the second through
the fourth cleanup. Per-cycle live snapshots also passed the proposed caps at
all five materialization and disposal points. No heap allowance was necessary
for the revised candidate's peaks. The detailed ledger gives counts, traffic,
headroom, both byte measures, and positive charges by workload.

The audit uses the larger of two complementary sentinel-pattern measurements.
It covers the full observed Rust callback, including the assertions and observer
frames, on a guarded 2 MiB worker stack. Thread startup, final thread teardown,
and output formatting are outside this boundary. The allocator measures Rust
global allocation requests and glibc usable bytes. RSS, concurrent workers,
other allocators and deployment-specific stacks need their own evidence.
The original diagnostic build warned about its C pointer declaration. The
corrected probe uses an opaque pointer and passes strict Clippy in every
measured build configuration.

The revised final Hyperpath benchmark occupies 13,312,792 bytes, eight fewer
than baseline. Its text section is still 1,088 bytes larger and its union of
defined function ranges is 1,020 bytes larger. The exception-table section
fell from 946,740 bytes in the previous extraction to 939,936 in the revision.
The fixed program revision rebuilt to the same owner binary as primary.

The downstream native release check exercised Alumina's complete diagonal G1
fixture through schedule certification, canonical transcript validation,
rejected lowering, successful lowering, executor preflight and destruction.
All nine processes passed the original assertions. In 36 measured cycles,
all three variants reached the same maximum heap (3,834,427 requested and
4,109,232 usable bytes) and four-cycle traffic (176,181,848 requested and
195,424,440 usable bytes). Cleanup stabilized after cycle two in each run.
In the first repetition the previous candidate had two extra allocations at
the transcript checkpoint. The revised candidate had two at schedule and
transcript checkpoints, for 144 requested and 160 usable bytes at each.
These process-wide measurements include a single-test harness. Attribution of
these small differences remains open. The first probe build used a crate-local
unsafe allocator and failed the application's unsafe-code lint. The successful
runs use the same observer as an external test dependency.

Both benchmark images completed all 194 smoke cases. Below are changes in
Criterion mean duration for the revised candidate relative to baseline. The
two matched blocks used ABBA and BAAB order on CPU 0 with ASLR disabled.

| Hyperpath workload | ABBA | BAAB |
| --- | ---: | ---: |
| Axis-aligned offset | +0.36% | -0.62% |
| Quadratic hodograph | -3.32% | -2.98% |
| Pad clearance | -2.59% | -2.59% |
| Additive bead schedule | -1.00% | -1.19% |
| Exact trace import | -2.60% | -2.67% |
| 256-record parser | +3.08% | +3.02% |

The 48 case runs cover 1,440 samples and 43,768,125 measured callbacks. Both blocks had adverse confidence-bound separation for `specctra_grid_route_text_parse_256`.

A separate release consumer timed complete planning, replay and cleanup without
allocator instrumentation. Six balanced permutations covered both policies and
three trace modes across all three grid sizes, with 9,720 samples of
236,520 completed calls in total. The original control compared
warmed active tracing with fresh reference output. Fresh processes for each
control restored exact report and count agreement. The timing observations
use warmed workloads. Ratios below compare the mean for each source within
each block, then the median across six blocks.

| Source | Lowest to highest median change | Domains with higher median | Slower paired blocks |
| --- | ---: | ---: | ---: |
| Revised Hyperpath wrapper | -2.54% to +1.63% | 7/18 | 40/108 |
| Direct Hyperdynamics | -5.39% to +2.30% | 7/18 | 45/108 |

For the direct entry point, all six paired blocks were slower on 4×3 strict passive (+1.38%), 4×3 approx passive (+1.60%).

The reproduced owner runtime regression rejects this revision under the
existing performance gate. The host memory proposal remains unadopted. The tables record positive charges against the fixed and current
baselines for these named workloads. Cross-target deployment headroom, full
runtime qualification, downstream final-image accounting and distribution
checks remain open. The revised source is on branch
`interval-axis-projection-wrapper-stack`. All 602 owner tests, strict Hyperpath
Clippy, formatting and strict external probe Clippy passed.

Raw evidence is in archive commit `cccd68301db8ae9e8f913d9d221b38f549d3642c`, record
`docs/evidence/2026-11-17-interval-axis-owner-memory`. The index binds the three archive
files to the verified payload. [Evidence index](../evidence-archive.tsv)
