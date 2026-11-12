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
