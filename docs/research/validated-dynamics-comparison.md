# External validated-dynamics comparison

The maintained [runner and acceptance contract](../../scripts/validated-dynamics/README.md)
compare Hyperdynamics with CAPD::DynSys on twenty exact initial-value problems.
Both complete every case and enclose the independent analytic endpoint and
whole-horizon extrema. The original eight-case campaign is recorded first; the
extended campaign below adds explicit time dependence, noncommuting Jacobians
and different decay rates. CAPD has lower median latency throughout the extended
campaign; enclosure quality depends on the case. This establishes
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

## Extended nonautonomous comparison — 2026-11-15

The expanded corpus preserves the original eight IVPs, certificates, resource
settings and quality thresholds. Each backend's original eight certificate
records match the same-revision baseline byte-for-byte, including Hyperdynamics
work counters and CAPD's signed IEEE zero. Mathematical kernels at
`0a4a09c6c11fe2bdf37cc6d59c4b9db393519bd4` remain unchanged. The archive binds
the extended adapter sources by hash; no new production dependency is introduced.

Each additional field has both point and box inputs, with the same independent
coordinate radius `2^-10`, horizon `[0,1/4]`, endpoint width limit `1/64` and tube
width limit `1`. The Taylor orders remain Hyperdynamics 3 and CAPD 12. Actual
steps are at most `1/16`, except the prespecified two-rate cap of `1/64`.

| Field | Point initial state | Independent exact solution |
| --- | --- | --- |
| `x' = t*x` | `1` | `x(t) = x₀ exp(t²/2)` |
| `x' = 1-8t` | `1` | `x(t) = x₀+t-4t²` |
| `(x',y',z') = (y,t*z,0)` | `(1,0,1)` | `x = x₀+t*y₀+t³*z₀/6`, `y = y₀+t²*z₀/2`, `z = z₀` |
| `(x',y') = (-x,-8y)` | `(1,1)` | `x = x₀ exp(-t)`, `y = y₀ exp(-8t)` |

The independent oracle bounds positive exponentials by a rational series and
geometric tail, then reciprocates these positive bounds for decay. It verifies
continuous extrema as well as endpoints. The turnaround field attains its
maximum at `t=1/8` and returns to its initial set at `t=1/4`, so an endpoint-only
tube is wrong. For the noncommuting field, the lower x envelope over box inputs
is `1-r-r*t+(1-r)*t³/6`. Its minimum occurs at `sqrt(2r/(1-r))`, inside the
horizon, and equals `1-r-(2/3)*r*sqrt(2r/(1-r))`. Exact integer-square-root bounds
enclose that event. Other coordinate extrema follow monotonicity.
The Jacobians satisfy `A(s)A(t)=t*E₁₃` and `A(t)A(s)=s*E₁₃`; replacing ordered
composition with an exponential of the integrated Jacobian gives the wrong
`T³/4` coefficient instead of `T³/6`. A corrupted-output control checks this case.

The final campaign uses the same host, CPU 0 affinity, disabled ASLR, compiler
versions and Hyperdynamics/CAPD/CAPD/Hyperdynamics block order as above. Ten
samples of 100 completed operations per solver/case/mode produce 640 samples
and 64,000 timed integrations. Every block independently validates the full
sixteen-case corpus before any timing summary is accepted. Preparation modes,
native evidence retention, materialization and destruction have the same cost
boundary as the original campaign. The table reports microseconds per operation.

| Case | Hyper reuse (us) | CAPD reuse (us) | Hyper fresh (us) | CAPD fresh (us) |
| --- | ---: | ---: | ---: | ---: |
| time growth point | 760.035 | 35.876 | 852.567 | 67.324 |
| time growth box | 759.204 | 35.574 | 850.962 | 67.242 |
| turnaround point | 603.497 | 29.869 | 672.231 | 63.603 |
| turnaround box | 598.580 | 29.887 | 672.276 | 63.962 |
| noncommuting point | 1760.247 | 79.852 | 2126.705 | 130.127 |
| noncommuting box | 1714.107 | 80.097 | 2075.937 | 130.364 |
| two rate point | 5015.614 | 229.668 | 5182.361 | 304.534 |
| two rate box | 5014.972 | 228.958 | 5183.075 | 303.040 |

The largest endpoint-coordinate widths are shown below, rounded only for
display. Exact rational widths and every positive contiguous time domain remain
in the raw records. All results meet the unchanged quality contract.

| Case | Hyper endpoint width | CAPD endpoint width | Hyper / CAPD pieces |
| --- | ---: | ---: | ---: |
| time growth point | 3.3006697e-07 | 2.6645353e-15 | 4 / 4 |
| time growth box | 0.0020154543 | 0.0020151238 | 4 / 4 |
| turnaround point | 0 | 0 | 4 / 4 |
| turnaround box | 0.001953125 | 0.001953125 | 4 / 4 |
| noncommuting point | 5.2939559e-23 | 8.8817842e-16 | 4 / 4 |
| noncommuting box | 0.0024464925 | 0.0024464925 | 4 / 4 |
| two rate point | 4.1398904e-05 | 3.3306691e-15 | 16 / 16 |
| two rate box | 0.014473501 | 0.0015210953 | 16 / 16 |

Hyperdynamics gives the tighter noncommuting point enclosure. Its uncertain
two-rate endpoint width is about `0.01447`, near the `0.015625` limit and wider
than CAPD's result. This identifies a concrete wrapping-control workload; it does
not establish a general stiff-system envelope or a resource-equivalence claim.
No threshold or initial uncertainty was reduced after measuring the results.

Validation includes nine maintained Python tests, 1,500 exact square-root
controls, the original nine corrupted-corpus controls plus eight new controls,
MSRV Rust Clippy with warnings denied, formatting, C++ warnings-as-errors, and
all 42 pinned CAPD native tests. New controls reject missed interior extrema,
assumed commutativity, wrong dimensions or time dependence, the wrong initial
state, the two-rate step-cap violation, and stopped results with an endpoint.
The runner requires every case/mode/sample exactly once, the configured iteration
count, and positive integer timing values. Optimized Python is refused because
it would disable assertion-based validation. Both runner gaps were reproduced
before their fixes.

The initial extension pilot is retained separately: its CAPD oscillator initial
zero serialized as `+0` rather than the original `-0`. Both denote the same
coordinate, but the adapter now preserves the original bytes. Only the corrected
`campaign` supplies the extended tables. All original fixtures and thresholds
remain present. The archive row `2026-11-15-interval-external-dynamics-extended`
in `docs/evidence-archive.tsv` binds the sources, commands, checks and raw results.
Full-stack, hosted/HIL and the remaining plan qualification are not established
by this benchmark extension.

## Manual affine-noise wrapping qualification

The existing `FlowNoiseState` / `certify_noise_step` path also completes the
point and box two-rate cases at order 3, with sixteen steps of `1/64` and two
retained noise columns. The Cartesian integrator supplies candidate boxes only;
each noise step independently re-certifies its actual incoming affine outer
state. The original initial data, horizon and common width limits above remain
unchanged. This is an existing manual composition capability, not an automatic
noise-controller implementation.

| Input | Coordinate | Cartesian endpoint width | Noise endpoint width |
| --- | --- | ---: | ---: |
| point | x | 6.37146198e-10 | 4.98832206e-10 |
| point | y | 4.13989040e-5 | 3.51031049e-6 |
| box | x | 0.00250786278 | 0.00152109578 |
| box | y | 0.01447350104 | 0.000267950531 |

The maintained `flow_noise_witness` test checks every generated endpoint and
continuous segment against independent rational exponential bounds. Both
coordinates decrease from positive initial intervals, so the bounds cover all
times between the segment endpoints. It materializes the hull of all sixteen
tubes, checks the full-horizon width limit, and requires stricter noise endpoint
widths: `(2^-29, 2^-17)` for the point input and `(2^-9, 2^-11)` for the box.
Source/proof identity, nonzero residuals, fresh residual inputs and subsequent
step consumption remain explicit. The rational series oracle is shared with
the existing adaptive-Taylor tests; their original assertions remain intact.

The noise chain consumes 625,855 graph-work units / 658,480,044 scalar-work units
for the point input and 625,821 / 734,947,748 for the box. Those counters cover
one shared noise-chain budget; preparation, Cartesian proposal generation and
tube publication have separate budgets. They are not a total CPU or heap bound.
An exploratory profile motivates further preparation/arithmetic work, but this
qualification does not supply a matched wall-clock comparison with the tables
above. Automatic noise orchestration, global sensitivity and wider workload
qualification remain open.

## Long-horizon tier — 2026-11-23

The corpus now has twenty rows. `oscillator_long` integrates `(x',y')=(y,-x)`
from `(1,0)` to `t=8` with step cap `1/8`. That is more than a full turn, so a
box method meets the wrapping effect and both coordinates must reach the
initial modulus. `riccati_long` integrates `x'=x²` from `1` to `t=1/2` with step
cap `1/16`. Each has a point and a `2^-10` box. The endpoint width limit stays
`1/64`; the tube limit is `4` because a full turn spans about `2`. The oracle
bounds `cos 8` and `sin 8` by alternating series with exact interval products.
Hyperdynamics runs this tier with correlated affine-noise evolution and its
default options: order 3, up to 16 columns, policy-grid rounding and a 16-bit
relative column cutoff. The original sixteen rows keep Cartesian evolution.

The benchmark tree of `92e91f5d0` (measured as `dd87532fa` before a rebase that
added only an unrelated Hypercurve doctest fix) and CAPD `37057e589` were measured on CPU 0 with ASLR disabled, on the same Xeon
E5-2680 v2 host, with Rust 1.95.0 and GCC 14. The run used 100 iterations,
5 samples per block and four Hyperdynamics/CAPD/CAPD/Hyperdynamics blocks,
which is 800 samples. The 600-second block timeout was raised to 4,000 seconds for this tier.
All 40 solver/case outcomes complete and pass the exact oracle in every block.
CAPD has the lower median in all 40 case/mode configurations, from 4.6 times
(Riccati point, fresh) to 1,258 times (long rotation box, reuse). CAPD's
upstream tests were not rerun; they passed at this pinned commit on 15 November.

| Long-tier row | Hyperdynamics reuse | CAPD reuse | Hyperdynamics endpoint width | CAPD endpoint width |
| --- | ---: | ---: | --- | --- |
| `oscillator_long_point` | 1,155,360 us | 931 us | 2.61e-4, 2.05e-4 | 1.27e-14, 1.78e-14 |
| `oscillator_long_box` | 1,170,221 us | 931 us | 2.50e-3, 2.47e-3 | 2.22e-3, 2.22e-3 |
| `riccati_long_point` | 38,152 us | 352 us | 8.84e-4 | 2.82e-14 |
| `riccati_long_box` | 49,074 us | 352 us | 8.71e-3 | 7.83e-3 |

Fresh preparation adds under 0.2% for the long Hyperdynamics rows. For the
boxes, Hyperdynamics' endpoints are 11–13% wider than CAPD's. For the points,
order-3 remainders over 64 and 8 steps dominate, against CAPD's order 12.
Both meet every limit. Before the rounding change accepted on 23 November,
correlated Hyperdynamics evolution of nonlinear fields stopped after two steps
with `SizeExhausted`; the Riccati long row is the first nonlinear workload in this
corpus that it completes. The rotation row shows a concrete cost gap of about
three orders of magnitude in correlated long-horizon transport. It does not
establish behavior beyond this corpus, at other orders, or under other options.
