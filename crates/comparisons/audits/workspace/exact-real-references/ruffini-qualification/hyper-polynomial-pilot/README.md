# Hyper polynomial transfer pilot

Status: isolated experiment; no new production changes. Full ecosystem audit
remains ACTIVE/OPEN. Timing, correctness and lifetime-aware allocation studies
are complete and validated. The additional coefficient-export oracle passes
4,905checks per debug/release mode without using Hyper equality for validation.
See `analysis.json` for machine-readable results.

## Provenance and numerical contract

Ruffini's polynomial three-product identity motivated this experiment, but its
Java thresholds and performance results were not transferred. `main.rs` is an
independent borrowed-slice implementation with leaf cutoffs8,16,32. The pilot
gate requires both operands to have at least32coefficients, lengths strictly
within a2:1ratio, all coefficients to be exactly rational, and at least75% of
each operand's coefficients to have canonical numerator or denominator bit
length at least512. This is an experimental gate, not an established optimum.

`freeze.mjs` copied393Rust/manifest/readme files from four in-scope crates into
`snapshot/`, verifying source hashes before and after copying. It recorded the
live worktree state, including concurrent/user rational-comparison edits.
`baseline.rs` is mechanically extracted, unchanged, from the frozen
`hypersolve/src/curve_resultant.rs`: actual multiplication and exact zero
predicate. Empty-product behavior and certified trailing-zero trimming remain
the baseline contract. Existing production and previous frozen evidence were
not edited. Other unchanged helpers/callers inspected include repeated cubic
discriminants, polynomial square-root extraction, Bezout resultants and Mobius
polynomial composition.

Debug and release each pass3,245checks with identical output:2,880integer
shape/seed/algorithm cases,270large rational cases,95radical/zero/trailing-degree
boundaries. The integer/rational expected convolution uses `num::BigRational`,
independently of the Hyper multiplication algorithms. All timed products also
undergo full exact-result verification, including length128families beyond the
initial grid. This is not a full downstream or whole-application qualification.

The separately versioned `coefficient-oracle` exports canonical signed numerator
and denominator from each actual rational coefficient, then compares using
`num::BigRational`, never Real/Rational equality. Expected convolution is also
independently computed with BigRational. It checks shapes, costs/densities,
fresh/disjoint algorithm ownership with repeated reuse, zeros and cancellation:
4,905cases per mode, with byte-identical debug/release output. The original
300-second debug cap expired after its2,880shape cases, with no mismatch; that
failure remains preserved. The unchanged debug binary subsequently completed
the full grid in674.615seconds under a1,200-second cap. Release took29.750seconds.
Those durations describe validation execution, not algorithm benchmark ratios.

## Timing and allocation methods

`run.mjs explore` preserves a preliminary32-family screen: fixed/reused inputs,
five algorithms,0.1CPU-second prewarm/calibration target, two rotated warmup
rounds and three measured rounds. It shares input cache ownership across
algorithms and is only exploratory; do not use it as production-retention proof.

`lifetime/run.mjs stable` runs ten shapes/cost/density families with fresh and
reused ownership separately, each in three independent seeded processes. It
compares baseline, an identical baseline function-pointer control, cutoff8 and
the cutoff16gated pilot. Reused inputs have disjoint ownership per algorithm;
fresh inputs reconstruct new RationalData rather than cloning shared Real
handles. Fresh pools are prepared in groups of eight outside timing, then
destroyed outside timing; product destruction and exact verification remain
inside the measured region. CPU/wall values for fresh runs sum those measured
regions, not total end-to-end process time.

Each process pins to CPU6 on a shared, non-exclusive host. Warmup/calibration
targets0.25CPU seconds, fixes batches before measurement, rotates algorithm
positions, and records two warmup plus six measured rounds. Any measured batch
below0.125CPU seconds rejects the process. Linux process CPU time is read with
clock_gettime, not coarse /proc ticks. Instrumented allocation executables are
separate and are forbidden in timing mode. No other owned heavy work was run
concurrently with this study. Across-process median ranges are descriptive;
they are not confidence bounds or universal crossover claims.

The original `run.mjs memory` failed its assumption that output destruction
releases everything allocated during multiplication: input arithmetic caches
retain results. It remains preserved, with its original source and binary.
The versioned `lifetime` allocator counts every allocation/deallocation for
the process lifetime. All72fresh/reused cases pass, four algorithm profiles
each; baseline and identical control accounting match exactly, and dropping
all measured input/output owners releases the associated allocations. Records
separate allocation calls/bytes, peak additional live bytes, and retained input
cache deltas after output destruction. They do not claim that peak memory
improves when cumulative allocation bytes fall.

## Completed timing results

All60processes passed:1,920observations,1,440measured batches,
7,453,824fully compared products; minimum measured CPU duration0.150768seconds.
All identical-code control process medians lie within10% of baseline. Selected
gated/baseline CPU ratios below give the median across three process medians,
with their range in parentheses—not a confidence interval:

| Family | Fresh inputs | Reused inputs |
| --- | --- | --- |
| length32,512bit,dense |0.824(0.818–0.859)|0.793(0.768–0.794)|
| length64,2048bit,dense |0.608(0.593–0.615)|0.570(0.557–0.573)|
| length128,2048bit,dense |0.462(0.439–0.465)|0.433(0.411–0.446)|
| length64,2048bit,fractional |0.611(0.608–0.622)|0.576(0.568–0.582)|

Wall ratios closely track CPU. Gated small/sparse/highly unbalanced fallbacks
remain near baseline on tested families; tiny differences must be interpreted
against control noise. Ungated cutoff8 also wins some small/sparse Hyper cases,
unlike the donor screen, and can outperform this conservative gate. Therefore
neither the donor's crossover nor the pilot gate is a proven general threshold.

Memory tradeoff, fresh length64dense2048bit example: gated cumulative allocated
bytes6,132,976versus9,574,040baseline, but peak additional live622,848versus
346,976bytes. Fewer allocations do not imply lower peak memory. Reused inputs
have different retained-cache deltas, explicitly reported in the allocation data.

The live `hyperreal/src/rational/arithmetic/comparison.rs` changed after this
snapshot was frozen. Its current source is not covered by these timings. The
historical aggregate also correctly rejects live comparison-source drift; neither
old nor new qualification is relabeled as applying to concurrent modifications.

Final checkpoint at2026-09-07T02:19:42Z: concurrent work restored comparison.rs
to its Git baseline8a4138fa..., while scalar_micro.rs and rational/tests.rs
changed again. The new pilot snapshot now differs in all three files. Historical
scalar/matrix/polynomial guards now reject rational/tests.rs instead. This is
recorded source drift, not a failure of the frozen numerical checks; it remains
mandatory to re-freeze and qualify current source before production retention.

## Retention gates still open

- Revalidate against current source before any production retention decision;
  the completed coefficient-export oracle covers only its frozen snapshot.
- Measure actual Hypersolve callers, including low-degree, sparse and symbolic
  geometry; isolated high-degree dense products do not establish user benefit.
- Compare a caller-specific alternative first: `exact_polynomial_square_root`
  recomputes a full square each backward coefficient step but consumes only
  coefficient `root_degree + power`. Directly accumulating that convolution
  diagonal could eliminate unused products while retaining the final exact
  squared-identity check. This is unimplemented/unqualified, not a retained fix.
- If generic dispatch still wins on useful callers, verify production tests,
  strict unknown/domain behavior, downstream suites, allocation limits, binary
  size and code-size cost before keeping it. No donor timing is a Hyper claim.

The outstanding Ruffini numeric resources/SVG and all other unaudited references
remain in scope; this pilot does not replace the exhaustive audit objective.
