# Solver interval-box proof scope

## Certified radius domain

Phase 7 belongs to Hypersolve. The public interval and Krawczyk certifiers accept
a candidate and a list of exact real radii. Each supplied radius must be
nonnegative before it can bound an enclosure. The witness concerns P1, P6,
P8, P9 and P19. It exercises the distinction between a policy-authorized
approximate decision and proof of an input precondition.

On `97fe4e072`, the sine addition identity makes the following radius exactly
negative, even though its terminal comparison with zero returns approximate
equality.

```text
sin(e + 1) - sin(e) cos(1) - cos(e) sin(1) - 2^-2048
```

All three residual packages certified the constant equality on that input.
The affine Krawczyk certifier for `x = 0` and both quadratic Krawczyk certifiers
for `x² + x = 0` returned `CertifiedUniqueRoot` at zero. The supplied interval
was invalid. Commit `e45bb6b62` restricts the four radius comparisons to
certified decisions. Both policies now return the existing
`UnknownVariableRadiusSign` diagnostic for the hidden negative. The public
box facade reports `InvalidInput` with no certificate. An ordinary negative
radius still receives its proved-negative diagnostic. Zero and positive
radii can be used on the same analysis and context after rejection.
[Regression tests](../../crates/hypersolve/tests/interval_box_radius.rs)

## Completed checks

| Check | Observed result |
| --- | --- |
| Rust 1.95.0 Hypersolve, all features | 1,523 tests and doctests passed, including the two-root and unequal-radius Krawczyk controls |
| Hypersolve, no default features | 1,388 tests and doctests passed |
| Hyperdynamics reachability | All 11 downstream integration tests passed |
| Static checks | Workspace formatting, owner all-feature/all-target Clippy with warnings denied, owner all-feature rustdoc with warnings denied, repository budget passed |
| Native release client | All 84 ordinary box replay records had identical verdicts, full evidence, dispatch traces and allocation observations |
| WASM release client | All 240 ordered calls per revision passed in two fresh instances with zero imports. Fixed baseline, previous primary and repaired versions reached 1,310,720 linear-memory bytes from 1,245,184 |

The native client covered two policies, three packages, two centers and seven
constraint and radius configurations. The separate native and WASM domain fixtures exercised
exact zero, one, negative one, a hidden negative, and its positive shift.
The native Criterion comparison exercised nine owner workloads, including
coupled Krawczyk systems through dimension eight. An exploratory timing run
during concurrent builds produced an outlier. In the four-block comparison,
affine Krawczyk appeared 2.54% slower. An independent eight-block pinned rerun
measured candidate medians of 3.842–3.919 µs and control medians of
3.869–3.872 µs. The larger affine slowdown did not repeat. The raw samples
are retained. No speed improvement is claimed.

## Bounded stack and linked size

The Linux qualification client ran one complete construction, proof attempt
and result inspection per call. Across 60 workloads, two temperatures, two
sentinel bytes and 24 fresh processes per revision, the largest **per-workload**
increment in observed stack was 32 bytes. The largest cumulative charge was
48 bytes. This includes observed increases between fixed baseline `c9f1f9ac5`
and the previous primary. After the pilot, the confirmation run used caps of
128 bytes and 1% per change, 256 bytes and 2% cumulative, separately for each
workload and temperature. The minimum remaining thread headroom across all
three builds was 2,072,776 bytes. Caller frames and the guard are accounted
for in headroom. The measured call includes nested solver work and cleanup.
The per-workload ledger debits increases even where process-to-process
variation contributed to a difference between maxima.
[Stack accounts and limits](interval-memory-accounts.json)

Peak requested heap across the campaign was 15,036 bytes on all three
revisions. Each pair of completed calls returned to zero live requested
and allocator-usable bytes. Maximum observed glibc usable peaks were 16,536,
16,504 and 16,504 bytes respectively. Requested bytes, usable bytes, allocation
traffic, process RSS and wasm linear memory are separate observations. The
qualification covers these serial host calls. It establishes no firmware or
arbitrary-input memory bound.

| Final linked image | Previous primary | Retained | Change |
| --- | ---: | ---: | ---: |
| Native owner certification benchmark | 16,928,328 | 16,928,288 | −40 bytes |
| Native replay observer | 3,790,984 | 3,791,000 | +16 bytes |
| WASM interval and Krawczyk client | 1,765,964 | 1,766,030 | +66 bytes |
| WASM code section | 1,569,561 | 1,569,586 | +25 bytes |

Every measured native allocated section and the union of defined function
ranges held its size. The native replay observer carries 56 bytes of positive
file growth from the fixed program revision. The WASM file and code charges
remain separate. The linked clients fit the 64 MiB host image allowance with
at least 16 MiB free. [Target size accounts](interval-size-accounts.json)

## Unresolved whole-box certificates

The public facade also aggregates row results that apply only at the center.
For example, `x³ = 0` holds at zero and fails at one. Both lie in `[-1,1]`
and `[0,2]`. On the previous primary the corresponding centers yielded
`Certified` and `Violation` respectively. Neither aggregate verdict describes
the whole box. A separate fixture reproduced a certified aggregate from an
approximate ball receipt. Both issues remain in the retained implementation.

The review branch `interval-box-replay-coverage-review` at `245d4bb43`
preserves a broader candidate and its six regressions. It passed 1,527
all-feature owner tests and doctests. Across the 84 native rows it corrected
24 aggregate verdicts while preserving all inner evidence, dispatch and heap
records. Its linked client's allocated RELRO padding grew from 1,376 to
1,992 bytes and a sparse alternative grew it by 800 bytes. Both exceeded the
existing 1% per-section cap. The smaller file and function union cannot
pay for another section's growth.

The archived commands, fixed/control/candidate executables, failed fixtures,
per-workload observations and raw timing samples are indexed under
`2026-11-18-interval-box-radius`. [Qualification archive](../evidence-archive.tsv)

## Exact extrema qualification and composed repair review

On `67a8dfeaf`, four of the five whole-box regressions failed. Checking only
the outer box status also leaves a false inner count: a real approximate
ball receipt contributed one certified satisfied row and zero uncertain rows.
Correcting those counters exposed an existing diagnostic regression that
discarded the approximate row. The four regression tests from the standalone
residual-certainty repair reproduce this difference. The composed review
retains the original row receipt through the diagnostic.
[Earlier residual-certainty qualification](residual-certification-ownership-review.md)

The maintained test in `cf261b302` checks 4,800 univariate-polynomial box
reports and 600 two-variable bilinear reports. For a quadratic it computes the
exact endpoint values and includes the vertex whenever its derivative changes
sign or vanishes at an endpoint. This includes a sole interior zero at 1/3.
For a bilinear function, affinity in each coordinate puts the extrema at the
corners of a rectangle, including rectangles of zero width on either axis.
Each certified sign and its interpretation for equality, soft and both
inequality directions must agree with the independent extrema. Singletons
and large positive and negative shifts must produce decisions. Removing the
quadratic remainder from either package made the test fail, and restoring
each remainder restored the passing result. The corpus qualifies the retained
polynomial packages on these cases. Point replay on unsupported residuals
remains a separate open defect. [Extrema tests](../../crates/hypersolve/tests/interval_quadratic_extrema.rs)

The composed candidate at `88beac671` produces summaries after residual-ball
replay, distinguishes certified and approximate receipts in the owning row
type, and checks whether each decisive row covers the supplied box. It keeps
source order and the effective last radius for a repeated symbol. The native
client corrected 24 of 84 verdicts. All 84 complete evidence/dispatch records
and all 84 allocation records match the control. A variant that recomputed
structural facts altered 42 traces. Another that carried a borrowed radius and
its certified zero flag saved 128 peak requested bytes on every client case.
The latter still exceeded the existing native size caps.

Both native images of `88beac671` use fewer defined function bytes. Their
allocated RELRO padding grows by 1,176 bytes in the replay client and 1,712
bytes in the owner benchmark. Both exceed the existing per-section cap.
The WebAssembly client shrinks by 10,155 file bytes and 8,720 code bytes
against the previous primary. Each revision completed 384 ordered calls
across two fresh instances, checking box status, inner counters and receipt
certainty. The observation fixture explicitly verifies that the constant
row has an affine enclosure and uses point replay in the quadratic packages.

A further composition carries the validated radius by reference, including
the certified zero flag, and counts each final replay row once. It passed
*both* native image gates on the previously measured targets. On the 84-row
observer it saved 128 peak requested bytes per case, retained all exact
evidence and traces, and corrected the same 24 verdicts. The observer file
grew by 896 bytes, its defined-function union by 517 bytes, and the owner
benchmark file shrank by 3,928 bytes.

Two other targets still block that composition at `305abbcdb`. The existing
WASM interval/Krawczyk client passed all 240 calls, but its file grew from
1,766,030 to 1,769,702 bytes and its code section from 1,569,586 to 1,572,612.
Each exceeds the 1,024-byte incremental cap. Across the native lifetime
client's 60 workloads, peak requested heap held or fell on every workload.
Eight of 120 cold/warm stack accounts exceeded the previously declared 1%
incremental cap. For example, a multivariate quadratic box call increased
from 3,446 to 3,494 bytes (1.39%). Each workload used the maximum across
24 processes and two sentinel patterns. No allowance was transferred from
the workloads that used less stack. The composition is preserved on
`interval-box-validated-radii-review`.

Keeping the existing owned scalar map in the final-row implementation also
passed all 240 calls in the existing WASM client. Its image shrank by 1,003
bytes and its code section by 1,668 bytes. Its peak requested heap passed the
previous limits on all 60 cases. On the native stack, 24 of 120 accounts
exceeded the incremental relative cap and 16 of those exceeded the cumulative
relative cap. For example, an affine report grew from 3,302
to 3,350 bytes (1.45%). This variant also remains unadmitted.

The earlier candidate remains on `interval-box-proof-and-diagnostics-review`. The individual size failures
remain recorded even when a different image shrinks. The primary branch
retains the independent extrema tests.

The primary all-feature owner suite passed 1,525 tests and doctests. The
isolated `88beac671` and `305abbcdb` suites each passed 1,535, including the
5,400-case corpus and the diagnostic regressions. Strict owner Clippy passed
on all three revisions. The
primary also passed the minimal-feature extrema and radius tests, release
execution of the new corpus, strict minimal-feature Clippy on the new
target, workspace formatting, all-feature rustdoc and all 11 downstream
reachability tests. The new test-only release runner occupies 4,388,536
bytes. Production source is identical on the two primary revisions.

The fixed, current and candidate WebAssembly images, reversible native ELF
deltas, controls, failed mutants, 84-row differentials and source patches
are recorded under `2026-11-18-interval-box-aggregation`.
[Evidence index](../evidence-archive.tsv)
