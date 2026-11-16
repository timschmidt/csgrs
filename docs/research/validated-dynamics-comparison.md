# External validated-dynamics comparison

The maintained [runner and acceptance contract](../../scripts/validated-dynamics/README.md)
compare Hyperdynamics with CAPD::DynSys on eight exact initial-value problems.
Both complete every case and enclose the independent analytic endpoint and
whole-horizon extrema. In the recorded configuration, CAPD has a lower median
latency in every row and narrower nonconstant point enclosures. This establishes
a concrete performance gap for the current Hyperdynamics path, not general
superiority across representations, resource contracts or dynamical systems.

The Hyper source is the clean accepted revision
`e4d12c6f91ea197cf1153d8c5f5ce626a6eeb6c1`; the pending path-dynamics extraction is
excluded from these measurements. No production mathematics changed. CAPD 6.1.0
is pinned at `37057e5899bccaa6cd5701057b7e6ea5e232ef8b`, using its native interval
backend in a separate process. Its [interval ODE documentation](https://capd.sourceforge.net/capdDynSys/docs/html/a05233.html)
describes the validated solver and retained solution-curve interfaces used here.
The pinned source, rather than the older online documentation version, controls
the recorded build and behavior.

All inputs and time boundaries are exact dyadics. The horizon is `[0,1/4]` and
every actual step is positive and at most `1/16`. Point initial states are `1`
or `(1,0)`; box states add coordinate uncertainty `[-2^-10,2^-10]`. Hyperdynamics
uses Taylor order 3 and its default grid `2^-64`; CAPD uses Taylor order 12 and
its default step controller. The common endpoint width limit is `1/64` and
the whole-tube width limit is `1`, per coordinate. The algorithms may produce
stronger enclosures, whose actual widths remain visible below and in raw data.

## Recorded latency

Microseconds per completed operation, median of ten samples of 100 operations
for each row/mode. Two blocks per solver run in Hyperdynamics/CAPD/CAPD/Hyperdynamics
order, pinned to CPU 0 with ASLR disabled on an Intel Xeon E5-2680 v2 host.
The Rust compiler is 1.95.0; GCC 14 and CAPD's `-O2 -frounding-math` compile the
external adapter. Build/test work completed before timing. Every operation
materializes endpoint and tube output, retains its native evidence, and destroys
that output/evidence before the timer stops. Serialization and analytic oracle
checks are outside the timer. Both adapters check their structural completion and
step durations inside it. Raw samples and compiler/build flags are archived.

| Case | Hyper reuse | CAPD reuse | Hyper fresh | CAPD fresh |
| --- | ---: | ---: | ---: | ---: |
| constant point | 299.773 | 29.393 | 311.527 | 58.778 |
| constant box | 287.713 | 27.687 | 308.325 | 59.054 |
| exponential point | 519.166 | 32.175 | 557.514 | 49.645 |
| exponential box | 510.894 | 32.026 | 558.506 | 49.836 |
| Riccati point | 859.172 | 155.590 | 939.651 | 205.255 |
| Riccati box | 864.080 | 155.935 | 946.496 | 205.245 |
| oscillator point | 1191.731 | 55.738 | 1333.611 | 100.494 |
| oscillator box | 1186.522 | 55.550 | 1340.585 | 100.201 |

Reuse excludes field/solver/coefficient preparation. Fresh includes it. These
are warmed process measurements, not first-process startup costs. There are
320 raw timing samples, representing 32,000 completed timed integrations. The
full native proof objects differ: Hyperdynamics retains exact source-bound graphs
and rejected attempts, while CAPD retains its solution curve. The comparison
requires the common mathematical result; it does not assert interchangeable
proof serialization or equal resource/cancellation APIs.

## Endpoint quality

Largest coordinate width, rounded here only for display. The oracle uses exact
rationals derived from raw dyadic/IEEE bounds. Both constant cases have identical
widths. Hyperdynamics uses four accepted segments in all cases; CAPD uses four
except Riccati, which uses twelve. The whole-tube oracle checks continuous
extrema, including the oscillator's interior maximum, not selected samples.

| Case | Hyper width | CAPD width |
| --- | ---: | ---: |
| constant point | 0 | 0 |
| constant box | 0.001953125 | 0.001953125 |
| exponential point | 2.156908e-7 | 2.6645353e-15 |
| exponential box | 0.0025080783 | 0.0025078621 |
| Riccati point | 7.231771e-5 | 7.9936058e-15 |
| Riccati box | 0.0035452773 | 0.0034744842 |
| oscillator point | 1.6876511e-7 | 1.3322676e-15 |
| oscillator box | 0.0025080318 | 0.0023756179 |

## Acceptance and limits

All sixteen solver/case outcomes pass independent analytic containment, width,
input and horizon checks. Outputs stay identical within each adapter across
fresh/reused preparation and every measured block. Nine corrupted-corpus controls
are rejected, including an endpoint used as a whole tube, an oversized step,
wrong inputs/time, missing/duplicate cases, and a too-wide result. Exact square-root
controls cover 1,500 rational inputs; three series identities at zero also pass.
The standalone Rust adapter passes MSRV Clippy with warnings denied and formatting;
the C++ adapter passes GCC warnings-as-errors with external headers marked as system
headers. CAPD's complete native test build passes all 42 tests.

Earlier adapter campaigns are retained as ineligible for the declared configuration:
CAPD's time-map implementation overwrote a maximum-step setting, producing two
pieces in six cases. Exact checkpoints and checks of every piece repair that
adapter error; only `campaign-step-cap` supplies the tables above. A metadata key
collision in the first campaign was corrected; explicit timing commands always
recorded CPU 0. An early upstream test invocation raced its final executable link;
the complete post-build rerun passed. No test or width threshold was weakened.

The upstream CAPD build emits a GCC array-copy warning in its static-array
initialization test. That test is separate from this adapter's dynamically sized
interval vectors. It was not edited or silently suppressed; the build log is
retained. Passing these tests is not a universal audit of the external library.
The external source and Boost packages are recorded by revision/package hashes,
not copied into the project or evidence archive.

This corpus does not establish stiff/long-time behavior, resource-exhaustion
equivalence, high-degree symbolic support, allocation superiority or universal
completeness. The binary sizes are recorded but are not old/new refactor gates:
the compared programs implement different proof/report formats. Broader competitive
workloads, profiling the measured Hyperdynamics costs, and the remaining plan phases
are still open. The archive row for `2026-11-15-interval-external-dynamics` in
`docs/evidence-archive.tsv` binds the source, logs and raw measurements.
