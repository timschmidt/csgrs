# Performance and resource ledger

No consumer migration is qualified yet. The
primitive relations and arithmetic allocate nothing: their only storage is an optional pair
of primitive endpoints, and they call a fixed number of integer comparisons.
Production source forbids unsafe Rust. The primitive feature subset imports
neither alloc nor std; optional host backends have separate dependency graphs.
Cargo feature closure, tests and layout checks must validate that claim.

On a target with i64 alignment 8, the explicit-empty carrier uses 12 bytes
for i32 endpoints and 24 bytes for i64 endpoints. Alumina's nonempty Q30
wrapper is 8 bytes; therefore this carrier must not silently replace that
wrapper's layout. Delegation and retained representation need measured review
in phase 9. Source sharing alone is not evidence of improved firmware cost.

Baseline commands, raw outputs and pending gates are recorded in the root
progress record. No speedup, firmware cycle count, or binary-size improvement
is claimed by the skeleton.

## Phase 2 focused Q30 probe

The maintained `conformance/fixed_cost.rs` client materializes both endpoints
for the same four successful inputs. It includes construction, uses out-of-line
operation wrappers and alternates legacy/shared sample order. Rust 1.88 release,
x86_64 Linux, 30 samples/backend/operation, 65,536 completed operations/sample,
and separate warmup; the firmware comparison source is
`bdee397d0b80841f16bf17e7bcd029b4feb2c212`. Exact differential tests precede timing.
Raw output and hardware/toolchain details are retained in the evidence record
linked from the root progress ledger.

The first candidate kept directed quotient helpers out of line. It was rejected
as a performance candidate: median shared/legacy ratios were 2.044 for multiply,
1.301 for divide and 3.004 for square. Inspection found dynamic i128 division
instead of constant-scale specialization. Coverage work was also active during
that first exploratory probe, so it is not a controlled cross-run speedup claim.
The corrected candidate marks those two helpers inline, preserving every
arithmetic branch and operation. With coverage finished, the paired probe was:

| Operation | Legacy median ns/op | Shared median ns/op | Shared / legacy |
| --- | ---: | ---: | ---: |
| Add | 21.233 | 19.167 | 0.903 |
| Subtract | 18.126 | 16.429 | 0.906 |
| Multiply | 33.980 | 20.185 | 0.594 |
| Divide | 114.711 | 86.655 | 0.755 |
| Square | 22.300 | 17.799 | 0.798 |

These are finite probe medians, not general performance guarantees or evidence
of FOC timing equivalence. The shared five-operation wrapper is 1,086 text
bytes versus 488 for the legacy wrapper; callees differ, so those symbol sizes
alone do not compare complete backends. The combined harness has 346,782 text
bytes, 16,208 data bytes and 296 BSS bytes. Its initial non-inline version had
347,526 text bytes. No firmware binary/cycle/stack or HIL result is inferred;
those remain migration gates. Host cycle counters were not available here.

## Phase 3 host carrier probe

`conformance/host_cost.rs` compares the retained Hypersolve product formula to
the rational carrier on positive, mixed-sign, and wide-rational inputs. Both
paths consume validated intervals and materialize owned exact endpoints.
The probe uses Rust 1.95 release, 4,096 warmup pairs, 30 alternating samples
per backend/case and 16,384 completed operations per sample. It measures warm
arithmetic, not constructor cost or a whole solver workload.

The initial cross-crate call boundary measured shared/legacy median ratios
1.015, 1.040 and 1.038 while other qualification jobs ran. After inlining the
carrier's product and endpoint boundary, a separate run measured 1.004, 1.006
and 0.990. Raw samples from both runs are retained. These small differences
do not qualify a solver migration or establish a general speedup/no-regression
claim. Fresh-process dispatch tests separately require identical paths and
rational reducer counters for 450 recorded windows.

Dyadic arithmetic reserves upper bounds on magnitude bits before alignment,
products, division and rational conversion. Linear stages charge `n` units;
quadratic stages charge `n*n`, multiplied by the method's stage count. These
conservative reservations are the declared accounting contract; cancellation
can still consume a reservation. They are not measured CPU/heap bounds.
Constructors normalize caller-owned mantissas, and arbitrary-precision rational
arithmetic/Hyperreal evaluation have no new total-work limit. No independent
cache or worker pool is introduced. Resource errors publish no partial result;
fresh-budget retries are exercised by the maintained corpus.

## Strict IEEE core and validation correction

The integer backend's statically bounded u128 operations allocate no memory.
Power uses at most 32 squaring and 32 product stages per source endpoint, each
evaluating both directed bounds. Optional exact bridges have their separately
documented allocation/resource contracts.

On x86_64 Linux, Rust 1.99 release, `conformance/float_cost.rs` completes
affine → square → add → reciprocal and materializes both bounds of [4/5,1]
outward. Thirty samples of 16,384 chains have median 574.681 ns for f32 and
659.598 ns for f64. These are focused host observations, without an explicit
warmup or a competing certified backend. Carrier layouts are 12/24 bytes.
The complete probe executable has 358,621 text, 14,032 data and 2,210 BSS bytes;
runtime/formatting code is included, so this is not the kernel's isolated size.

An out-of-line finite validator prevents LLVM from substituting a native float
zero comparison that erases subnormals under DAZ. Compared with `385006d68`,
three alternating process rounds of the unchanged proposal probe (30 samples
of 16,384 completed steps per round) give these median ns/step:

| Width | Before correction | After correction |
| --- | ---: | ---: |
| f32 | 83.475 | 95.786 |
| f64 | 67.681 | 99.816 |

Output checksums agree for this ordinary workload. The proposal executable's
text grows from 332,894 to 334,058 bytes; total text/data/BSS changes from
350,272 to 350,268 bytes because section layout differs. This adverse latency
and text-size result is retained with the correctness fix; it is not a
behavior-preserving simplification or a general performance claim. Commands,
flags, hardware, binary digests, original failing release evidence and raw
samples are in the strict-float archive linked by the root progress record.

## Elementary order-image probe

`conformance/order_cost.rs` measures completed `abs → min_image → max_image`
chains on preconstructed inputs. It independently checks the expected result
before timing, warms up with 4,096 calls, then takes 30 samples of 16,384 chains,
owning and dropping each materialized result. On x86_64 Linux, Rust 1.99 release:

| Family | Median ns/chain |
| --- | ---: |
| Integer i64 | 8.666 |
| Fixed i64, FRAC=16 | 6.384 |
| Rational | 153.348 |
| Dyadic, with shared per-chain budget | 676.350 |
| f32 | 27.993 |
| f64 | 21.078 |

These families use different grids and representations; the table describes
individual completed workloads, not comparative backend speedups or firmware
cycle counts. No prior implementation of these public operations was replaced.
Inputs are small and fixed; allocation/large-coefficient behavior is not inferred
from this timing probe. Raw samples, whole-executable size and toolchain/hardware
identity are in the elementary-order archive referenced by the progress record.

## Rational signed-power probe

`conformance/rational_power_cost.rs` completes `checked_pow(3)` followed by
`checked_pow(-2)` on positive, negative and singleton inputs. Preconstructed
intervals are outside timing; 4,096 warmups precede 30 samples of 8,192 chains
per case. Timed calls consume owned exact endpoints. Independent rational
expectations are [4096/15625,4096/729] for the interval cases and 729/4096 for
the point case. Rust 1.99 release on x86_64 Linux measured median 561.891,
548.480 and 316.054 ns/chain, respectively. This is an ordinary completed-work
probe, not a general speedup, large-exponent or firmware timing claim.

The exact-power kernel keeps its existing eager-growth guard. Interval
selection avoids powering unused endpoints in positive-even zero-crossing
images; the reproduced initial failure and both timing rounds are retained.
No caller-selected total-work, heap-byte or cancellation bound is added to
Hyperreal by this adapter. The compiled host layouts are 1 byte for IntervalError,
16 for RationalInterval and 24 for its Result; these are observations, not ABI
promises. The complete probe executable has 666,956 text, 210,984 data and 1,818
BSS bytes, including runtime and scalar dependencies. Raw evidence is linked
from the root progress record.

## Square-root completion probe

`conformance/sqrt_cost.rs` materializes owned dyadic enclosures and coverage
receipts from preconstructed rational/dyadic inputs. The four inputs are the
singleton 2, narrow [2, 2+2^-16], wide [0,16], and partial-domain [-1,4].
Independent GMP tightness inequalities and directed MPFR bounds run before
timing. Rust 1.99 release on x86_64 Linux, rational+dyadic features, 1,024
warmups and 20 samples of 1,024 completed images per case/grid/backend:

| Backend / input | grid 2^-16 | grid 2^-64 | grid 2^-256 | grid 2^-1024 |
| --- | ---: | ---: | ---: | ---: |
| dyadic / point | 505.123 | 1178.072 | 2804.901 | 8326.062 |
| dyadic / narrow | 713.662 | 2263.782 | 5425.439 | 16532.276 |
| dyadic / wide | 367.915 | 683.699 | 923.493 | 2588.463 |
| dyadic / partial | 368.306 | 680.600 | 929.400 | 2606.833 |
| rational / point | 459.582 | 1219.851 | 2906.726 | 8366.091 |
| rational / narrow | 899.005 | 2681.588 | 5937.013 | 17435.620 |
| rational / wide | 397.708 | 735.685 | 975.101 | 2686.746 |
| rational / partial | 396.959 | 736.199 | 1004.698 | 2719.616 |

Values are median ns/image. Timed work includes a fresh deterministic budget,
owned output construction and destruction. Source parsing and oracle work are
excluded. All 640 samples complete with exact directed output bounds and an
explicit domain receipt. Raw samples, compiler/hardware metadata, executable
hash and whole-probe size are retained in the square-root evidence record.
The executable also includes the independent GMP/MPFR oracle; its size is not
a production-kernel size estimate. These are finite host completion costs,
not a consumer speedup, no-regression or firmware timing claim. Input width
alone does not determine cost: exact zero and perfect-square endpoints can
be cheaper than narrower nonsquare intervals.

Root extraction reserves quadratic accounting stages logarithmic in the
scaled quotient bit length, plus existing scaling/division and rational
cloning reservations. These are deterministic work/size units, not CPU cycles
or peak retained/temporary bytes. Allocation accounting, cancellation, full
coverage and consumer qualification remain separate work.

## Primitive square-root completion probe

`conformance/primitive_root_cost.rs` consumes preconstructed exact inputs:
point 1/4, narrow [1/4,1/2], wide [0,3/4], and partial-domain [-1/4,1/2].
Each run returns and consumes the complete range and domain receipt. Independent
GMP/MPFR checks precede timing. Rust 1.99 release on x86_64 Linux, fixed and
float-enclosure features, 4,096 warmups, 30 samples of 8,192 images per case:

| Endpoint / scale | Point | Narrow | Wide | Partial domain |
| --- | ---: | ---: | ---: | ---: |
| i32/8 | 25.323 | 40.549 | 29.953 | 29.959 |
| i32/16 | 36.830 | 62.811 | 41.528 | 41.526 |
| i32/30 | 45.795 | 89.795 | 58.151 | 58.148 |
| i64/16 | 34.853 | 62.886 | 40.892 | 40.952 |
| i64/30 | 45.362 | 89.241 | 57.985 | 57.999 |
| i64/62 | 67.633 | 155.899 | 101.671 | 101.671 |
| f32 | 80.796 | 156.981 | 93.167 | 93.164 |
| f64 | 100.867 | 218.976 | 136.015 | 129.738 |

Values are median ns/image. All 960 samples completed. Fixed scale F means
spacing 2^-F; IEEE uses its own magnitude-dependent grid. Thus cross-row times
are costs of different endpoint contracts, not speedup comparisons. Source
construction/oracle work is excluded; complete output materialization is
included. Raw samples, environment, executable hash and whole-probe size are
in the primitive-root evidence record. The executable includes GMP/MPFR test
oracles, so its size is not the production-kernel size.

Primitive implementations use at most two u128 integer root calls, with at
most 126-bit fixed or 106-bit IEEE radicands. Their production dependency
closure and source contain no allocation path. Firmware cycle/stack and HIL
measurements remain open. Release tests cover all 16 native rounding/FTZ/DAZ
combinations, and emitted code scans on Rust 1.88, 1.95 and stable, plus stable
fat LTO, find no native floating arithmetic/comparison in the selected root
and existing strict-float paths. This target/compiler evidence does not
qualify a different platform or replace the exact mathematical argument.

## Budgeted exponential completion probe

`conformance/exp_cost.rs` computes tight grid images of preconstructed exact
inputs: point 1, narrow [1,1+2^-8], wide [-4,4], and negative [-16,-8]. The
output includes both owned dyadic endpoints and Complete domain coverage;
all refinement attempts remain charged to the same budget. Independent
directed MPFR checks run before timing. Rust 1.99 release, x86_64 Linux,
rational+dyadic features, 128 warmups and 20 samples of 128 images per case:

| Backend / input | grid 2^-16 | grid 2^-64 | grid 2^-256 |
| --- | ---: | ---: | ---: |
| dyadic / point | 18646.520 | 36383.531 | 114201.234 |
| dyadic / narrow | 38583.367 | 69556.871 | 213629.574 |
| dyadic / wide | 64542.234 | 133154.922 | 452603.320 |
| dyadic / negative | 19836.590 | 75867.113 | 220062.328 |
| rational / point | 18753.453 | 36558.797 | 114525.246 |
| rational / narrow | 35268.566 | 69941.078 | 215594.012 |
| rational / wide | 64724.559 | 133569.125 | 453385.070 |
| rational / negative | 20020.934 | 76152.012 | 219971.211 |

Values are median ns/image; all 480 samples completed. Timed work includes
budget creation, Taylor tails, range reconstruction, any endpoint refinement,
output materialization and destruction. Inputs/oracles are excluded. Raw
samples retain deterministic work units per image alongside elapsed time;
compiler/hardware metadata, executable hash and whole-probe size are archived.
The executable includes GMP/MPFR oracle code, so its size is not kernel size.
These are finite host completion costs, not a speedup comparison to an
unbudgeted point evaluator or evidence of a consumer/firmware migration.

The ordinary corpus additionally completes values through ±1024, precision
through -2048, rational coefficients beyond 1024 bits, tiny inputs, and
extreme-scale negative tails under its declared size/work settings. Positive
extreme scales and small budgets exercise typed refusal, not completion.
Deterministic accounting is distinct from precision, actual heap bytes,
wall-clock time and cancellation. A broad memory/cancellation contract and
full consumer performance qualification remain open.


## Budgeted logarithm and extended-carrier completion probe

`conformance/log_cost.rs` computes tight grid images of preconstructed point
3/2, narrow [3/2,3/2+2^-8], wide [1/4,4], partial [-1,4], and near-one
singleton 1+2^-1024. Partial input produces a lower-unbounded image with
Partial domain coverage; other cases produce finite images with Complete
coverage. Independent directed MPFR/GMP checks precede timing. Rust 1.99
release on x86_64 Linux, rational+dyadic with no default features, 128 warmups
and 20 samples of 128 completed images per case:

| Backend / input | grid 2^-16 | grid 2^-64 | grid 2^-256 |
| --- | ---: | ---: | ---: |
| dyadic / point | 16662.770 | 42848.203 | 175941.465 |
| dyadic / narrow | 34742.832 | 86982.109 | 317726.281 |
| dyadic / wide | 45389.445 | 111702.309 | 461992.891 |
| dyadic / partial | 22422.578 | 55547.957 | 246855.066 |
| dyadic / near_one | 3852.109 | 3859.133 | 3898.789 |
| rational / point | 16714.676 | 42820.289 | 175344.684 |
| rational / narrow | 34843.965 | 84980.727 | 316127.480 |
| rational / wide | 45580.406 | 111499.133 | 460823.789 |
| rational / partial | 22357.762 | 61424.336 | 229865.598 |
| rational / near_one | 3895.039 | 3919.840 | 3929.492 |

Values are median ns/image; all 600 samples completed. Timing includes budget
creation, all series/refinement work, owned extended output, coverage receipt
and destruction; it excludes input construction and oracle work. The near-one
case benefits from retaining the exact source-side sign through logarithm
monotonicity, without needing to resolve its sub-grid distance from zero.
The original source remains exact. Every group uses a fresh 16384-bit,
10^12-work-unit budget per image; raw samples retain actual deterministic
work units alongside elapsed time.

The measured dyadic/extended layouts are 72/80 bytes before heap allocations.
The oracle-linked executable's text/data/bss sizes are 1090614/19304/3098 bytes;
these are not production-kernel size measurements. Raw samples, compiler and
hardware metadata, executable hash and coverage diagnostics are in the
logarithm evidence record linked by INTERVAL_REFACTOR_PROGRESS.md. These are
shared-host completion measurements, not a matched speedup comparison, a
consumer migration, or a bound on heap bytes, wall time or cancellation.


## Budgeted arctangent completion probe

`conformance/atan_cost.rs` computes tight grid images of point 3/2, narrow
[3/2,3/2+2^-8], wide [-4,4], huge singleton 2^4096, tiny singleton 2^-1074,
and the entire real line. All images are finite and have Complete local
coverage. Independent directed MPFR/GMP bounds precede timing. Rust 1.99
release, x86_64 Linux, rational+dyadic with no default features, 128 warmups
and 20 samples of 128 completed images per group:

| Backend / input | grid 2^-16 | grid 2^-64 | grid 2^-256 |
| --- | ---: | ---: | ---: |
| dyadic / point | 40947.449 | 104065.426 | 372543.457 |
| dyadic / narrow | 147121.023 | 188525.699 | 748553.340 |
| dyadic / wide | 77772.586 | 195476.242 | 772103.141 |
| dyadic / huge | 24293.031 | 55517.117 | 212433.848 |
| dyadic / tiny | 158.074 | 157.836 | 157.734 |
| rational / point | 41377.621 | 103905.637 | 373147.750 |
| rational / narrow | 147317.836 | 188380.363 | 747789.648 |
| rational / wide | 78034.375 | 194400.645 | 770841.938 |
| rational / huge | 24448.875 | 55619.367 | 212067.238 |
| rational / tiny | 211.426 | 211.734 | 211.703 |
| extended / entire | 22862.797 | 54400.172 | 210605.238 |

Values are median ns/image; all 660 samples complete. Source construction and
independent oracles are outside timing; budget creation, argument reduction,
series/tail work, every refinement attempt, output, receipt and destruction
are inside. Each call has a fresh 16384-bit, 10^12-work-unit budget. Raw samples
retain deterministic work units; compiler/host metadata, whole-executable size
and hash are in the arctangent evidence record. The executable includes
GMP/MPFR oracles and its size is not the kernel size.

The tiny case uses 0<atan(x)<x to avoid unnecessary evaluation below a grid
unit. Large dyadic inputs use reciprocal reduction without expanding their
scale; large rational input coefficients still have their own reservation
cost. All finite and unbounded cases preserve the same outward-grid contract.
These are new-capability costs, not baseline speedups or consumer benchmarks.
No heap-byte, wall-time, cancellation or firmware-cycle bound is claimed.


## Budgeted periodic sine/cosine completion probe

`conformance/trig_cost.rs` computes tight outward grid images of point 1,
narrow [1,1+2^-8], extremum-crossing [1,2], wide [-4,4], huge singleton 2^128,
tiny singleton 2^-1074, and the entire real line. Every result has Complete
local domain coverage. Directed MPFR with exact GMP input-error and extrema
checks runs before timing. Rust 1.99 release, x86_64 Linux, rational+dyadic,
no default features, 128 warmups and 20 samples of 128 images per group:

| Backend / function / input | grid 2^-16 | grid 2^-64 | grid 2^-256 |
| --- | ---: | ---: | ---: |
| dyadic / sin / point | 49288.188 | 88050.844 | 302846.449 |
| dyadic / sin / narrow | 64364.316 | 120516.570 | 391360.309 |
| dyadic / sin / extrema | 65831.676 | 131730.438 | 426930.289 |
| dyadic / sin / wide | 268.496 | 268.586 | 268.812 |
| dyadic / sin / huge | 212008.879 | 277929.414 | 599801.562 |
| dyadic / sin / tiny | 28612.375 | 60341.516 | 221476.539 |
| dyadic / cos / point | 49327.988 | 90109.438 | 304005.941 |
| dyadic / cos / narrow | 63912.113 | 123630.125 | 394203.262 |
| dyadic / cos / extrema | 63297.684 | 135303.945 | 429640.105 |
| dyadic / cos / wide | 269.691 | 269.402 | 269.590 |
| dyadic / cos / huge | 211096.453 | 277780.344 | 603552.664 |
| dyadic / cos / tiny | 31047.480 | 62744.184 | 223880.574 |
| rational / sin / point | 49500.176 | 88004.793 | 302219.266 |
| rational / sin / narrow | 64955.094 | 121445.152 | 391633.469 |
| rational / sin / extrema | 66034.145 | 132259.945 | 426594.695 |
| rational / sin / wide | 381.492 | 381.996 | 381.211 |
| rational / sin / huge | 212277.254 | 278003.613 | 599797.211 |
| rational / sin / tiny | 28882.438 | 60692.332 | 221623.781 |
| rational / cos / point | 49379.000 | 90367.531 | 304077.215 |
| rational / cos / narrow | 64213.824 | 124203.094 | 395021.852 |
| rational / cos / extrema | 63608.023 | 135570.273 | 429609.348 |
| rational / cos / wide | 380.723 | 380.832 | 380.586 |
| rational / cos / huge | 211338.488 | 277952.852 | 603042.523 |
| rational / cos / tiny | 31348.887 | 63269.785 | 224222.594 |
| extended / sin / entire | 144.082 | 144.387 | 143.984 |
| extended / cos / entire | 143.660 | 143.422 | 143.551 |

Values are median ns/image; all 1560 samples complete. Timing includes budget
creation, phase and extremum selection, all constant/series/refinement work,
owned output, receipt and destruction; input construction and the independent
oracle are excluded. Each call has a fresh 32768-bit, 10^13-work-unit budget.
Raw samples retain actual deterministic work units. Host/compiler metadata,
executable hash and whole-probe size are archived; GMP/MPFR oracle code is linked
into this executable, so its size is not production-kernel size.

Wide and unbounded source cases prove a complete function range without costly
phase resolution. Huge narrow inputs pay for enough pi precision to retain the
exact source phase. No error or unresolved phase is counted as a completed
whole-range image. These are new-capability costs on a shared host, not speedup
comparisons or consumer benchmarks. Actual heap-byte, wall-time, cancellation,
firmware-cycle and full consumer qualification remain open.


## Extended division and split-carrier completion probe

`conformance/division_cost.rs` materializes the full normalized quotient image
and local domain receipt. The finite positive/negative cases divide [1,2] by
[3,4], or [-2,-1] by [-4,-3]. Pole cases use [1,2]/[-1,1], [1,2]/[0,3], and
zero/[-1,1]. Unbounded cases use the entire line or closed rays as indicated
in the probe. A four-corner/limit GMP oracle runs before timing. Rust 1.99
release, x86_64 Linux, dyadic only with no default features, 128 warmups and
20 samples of 1024 completed images per group:

| Input case | grid 2^-16 | grid 2^-64 | grid 2^-256 |
| --- | ---: | ---: | ---: |
| positive | 1619.940 | 1686.340 | 1833.387 |
| negative | 2000.655 | 2085.183 | 2260.948 |
| split | 942.878 | 943.143 | 943.705 |
| touching | 785.209 | 846.865 | 1003.092 |
| entire_denominator | 423.767 | 423.539 | 422.947 |
| zero | 607.738 | 607.665 | 608.629 |
| unbounded_numerator | 776.747 | 836.000 | 991.025 |
| unbounded_denominator | 440.372 | 440.058 | 440.208 |
| entire_numerator | 81.238 | 81.199 | 81.201 |

Values are median ns/image; all 540 samples complete. Timed work includes
budget creation, domain/sign decisions, exact fraction reduction, selected
quotients, normalized components, receipt and destruction. Input construction
and independent oracle work are excluded. Each call has a fresh 4096-bit,
10^11-work-unit budget; actual deterministic work units accompany every row.
Raw samples, compiler/host metadata, executable hash and whole-probe size are
in the split-division evidence record. The executable includes GMP oracle code
and its size is not production-kernel size.

The container adds no separate union allocation; backend BigInts still own
storage. On this host, DyadicInterval, ExtendedInterval and IntervalSet2 occupy
72, 80 and 160 inline bytes respectively, excluding owned storage. Layout and
source-size/work reservations are distinct from actual heap-byte bounds.
Exact dyadic quotients remain compact even at
the minimum output-grid exponent, while genuinely fine nondyadic results must
pay their coefficient growth. These are new-capability costs, not speedups
against the finite strict API, which intentionally has a different pole contract.
Full memory/cancellation, consumer and firmware qualification remain open.


## Rational exponent completion probe

`conformance/fractional_power_cost.rs` measures full normalized images and
local domain receipts. Cases are 2^(1/3), [2,3]^(3/5), [-8,-1]^(2/3),
[-3,5]^(-1/3), [-3,5]^(-3/2), [8,27]^(1/3), (2^512)^(1/3),
(2^-512)^(1/3), and the entire line raised to -1/3. Exact GMP power/root/grid
expectations run before timing, with an additional directed MPFR point check.
Rust 1.99 release, x86_64 Linux, dyadic only with no default features:

| Case | grid 2^-16 | grid 2^-64 | grid 2^-256 |
| --- | ---: | ---: | ---: |
| point | 12368.426 | 48927.836 | 222718.707 |
| narrow | 17094.371 | 58084.070 | 274537.297 |
| negative | 2205.164 | 2198.148 | 2197.098 |
| split | 10379.039 | 57336.215 | 210931.961 |
| partial | 4751.941 | 17984.922 | 61941.824 |
| perfect | 2904.707 | 2894.859 | 2841.676 |
| huge | 126160.676 | 166265.855 | 375193.273 |
| tiny | 1916.641 | 1914.168 | 55832.668 |
| entire | 609.613 | 609.582 | 609.684 |

Values are median ns/image. All 540 samples complete: three precisions, nine
cases, 20 samples of 128 calls after 32 warmups per group. Timed work includes
exponent normalization, root relations, arithmetic and work reservations,
component normalization, owned output/receipt and destruction; input construction
and the independent oracles are excluded. Each call starts a fresh 32768-bit,
10^13-work-unit budget. Raw rows retain actual deterministic work units.

Perfect rational roots avoid output-grid expansion; a nonperfect root uses
integer bisection with exact power comparisons. Huge and fine-grid cases pay
for their larger integer relations. Bounds and root witnesses are recomputed
per call; no warm cache is assumed. These are costs for a new operation, not
speedup comparisons. The archived executable includes GMP/MPFR oracle code,
so whole-probe size is not production-kernel size. Actual allocation/heap-byte,
cancellation, rational-backend timing and downstream consumer qualification
remain open. Source/work reservations do not establish those claims.


## Tangent and cotangent completion probe

`conformance/tangent_cost.rs` evaluates rational and dyadic point 1, narrow
[1,257/256], pole-sensitive [1,2], wide [-4,4], huge 2^128 and tiny 2^-1074
sources, plus the extended entire line. The [1,2] case crosses a tangent pole
and stays within one cotangent branch. Independent directed MPFR point bounds
and exact GMP pole indexing run before timing. Rust 1.99 release, x86_64 Linux,
dyadic+rational with no default features:

| Backend / function / case | grid 2^-16 | grid 2^-64 | grid 2^-256 |
| --- | ---: | ---: | ---: |
| dyadic / tan / point | 60158.875 | 112760.594 | 369536.562 |
| dyadic / tan / narrow | 93677.594 | 176096.219 | 541577.000 |
| dyadic / tan / pole | 90750.688 | 196303.438 | 612686.688 |
| dyadic / tan / wide | 154.625 | 154.125 | 154.438 |
| dyadic / tan / huge | 295284.031 | 383250.156 | 821884.406 |
| dyadic / tan / tiny | 30974.781 | 60508.219 | 214659.500 |
| dyadic / cot / point | 58076.969 | 112557.531 | 368712.656 |
| dyadic / cot / narrow | 93402.375 | 176021.156 | 538127.000 |
| dyadic / cot / pole | 90946.000 | 195939.406 | 608159.125 |
| dyadic / cot / wide | 154.438 | 155.031 | 154.344 |
| dyadic / cot / huge | 295125.438 | 382224.750 | 821193.156 |
| dyadic / cot / tiny | 34548372.438 | 57426317.531 | 39388858.375 |
| rational / tan / point | 58283.438 | 112676.406 | 368997.188 |
| rational / tan / narrow | 94043.562 | 176234.281 | 541215.188 |
| rational / tan / pole | 90898.250 | 195982.438 | 610121.188 |
| rational / tan / wide | 257.875 | 256.562 | 257.375 |
| rational / tan / huge | 294786.562 | 382600.031 | 820607.219 |
| rational / tan / tiny | 31151.125 | 60540.000 | 214032.844 |
| rational / cot / point | 58222.000 | 112481.250 | 367486.031 |
| rational / cot / narrow | 93797.969 | 176370.562 | 537880.656 |
| rational / cot / pole | 91055.188 | 195886.562 | 606496.750 |
| rational / cot / wide | 257.969 | 259.094 | 257.500 |
| rational / cot / huge | 295563.656 | 381423.125 | 819743.562 |
| rational / cot / tiny | 34490514.812 | 57398168.469 | 39366275.656 |
| extended / tan / entire | 17.000 | 17.000 | 17.000 |
| extended / cot / entire | 17.219 | 17.219 | 17.250 |

Values are median ns/image. All 1560 samples complete: 20 samples of 16 calls
following 16 warmups for each backend/function/source/precision group. Timed work
includes fresh 32768-bit/10^13-unit budgets, phase and pole proofs, refinement,
full normalized image/receipt materialization and destruction. Source construction
and independent oracle work are excluded. Actual deterministic work units remain
in every raw row; metadata records compiler, host, executable hash and size.
The executable links GMP/MPFR, so whole-probe size is not production-kernel size.

Tiny cotangent is a stress case: its large image lies very close to a grid
boundary, requiring substantially finer internal bounds than the requested
output precision. That cost is reported separately from ordinary point and
interval completion. Wide/unbounded sources prove complete branch coverage;
no undecided denominator is counted as whole-range success. These are costs
for new capabilities, not speedup claims. Actual allocation/heap-byte bounds,
cancellation, embedded cycles and full downstream qualification remain open.

## Directed dyadic quotient sharing: rejected size tradeoff

At `4d4e22a31`, `scaled_quotient` computes floor and ceiling with two backend
magnitude divisions. Two single-division candidates preserved exact outputs and
work/error contracts in focused tests. A same-source-path external consumer
exercised 15 completed division, conversion, root and elementary-function cases;
canonical outputs and domain qualifications matched, with no allocation-request
increase. This is not a completed timing or downstream qualification campaign.

Neither candidate passed all required size gates. With the workspace-derived
lock (num-bigint 0.4.6), a signed quotient/remainder candidate grew the executable
by 432 bytes; retaining unsigned magnitudes until directed output construction
reduced it by 1,224 bytes. With the external resolver's lock (num-bigint 0.4.8),
the corresponding changes were +1,008 and +104 bytes. Function-address unions
also grew for both external candidates. Both locks and all adverse results are
retained in the evidence record linked by the root interval progress ledger.
No dependency range, size gate, precision, fixture or work weight was narrowed.

Production arithmetic is restored. Independent directed-quotient tests retain
small signed-integer expectations, large-limb multiplication inequalities,
resource-admission precedence and a zero Hyperreal-dispatch assertion. The
original double division remains until sharing can satisfy the unchanged cost
gates across the demonstrated dependency configurations. No speedup or phase
completion is claimed from these tests or the rejected experiments.
