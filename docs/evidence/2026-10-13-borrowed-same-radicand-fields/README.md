The borrowed-field same-radicand version is **rejected for runtime admission**.
All 42 scalar qualification commands passed, but six phases are slower than
baseline in every fixed pair and fifteen in at least four. It preserves six
allocation-saving phases and reduces probe size relative to the outlined
control, with no additional measured memory saving. The [decision](decision-v1.json)
retains all favorable and adverse results. Main runtime is unchanged.

The three fresh sources are the unchanged endpoint/P4 parent, the rejected
outlined version, and a helper using checked field borrows. The [review](scope-v1.json)
establishes disjoint mutable borrows of the left constant, radical pair and
empty atom vector, plus shared borrows of the right constant and pair. The
caller retains the original opaque guard and singleton/equal/non-unit checks.
The helper adds no vector indexing, scalar comparison, clone, unsafe access,
public API or cache field.

On cancellation, the helper returns `Some(Rational)` without replacing input
fields. The caller constructs the allocation-free rational carrier and drops
the consumed input. On survival, the helper assigns the selected radicand,
constant, coefficient and empty atom vector in the original order. Atom-buffer
release remains inside the helper before remaining local drops. `None` means
the borrowed fields contain the one-radical product; the enclosing multiplier
still returns `Some` for every eligible product.

Reviewed substitutions preserve arithmetic order, zero skipping and operand
forms. Removing the helper/branch and receiver `mut` recovers the original
general multiplier exactly. Cache storage, memo publication, mandatory retry,
work/depth budgets and the stronger-facts P4 contract remain inherited.

Each frozen source contains 10,136 files. Baseline and outlined match their
previously qualified sources byte for byte; only the algebra file differs
across variants. All 28 shared tests, manifests and lockfiles match. The source
maps, inputs and deltas were committed before compilation.

The [qualification assessment](correctness-v1.json) verifies all 42 commands,
logs and source bindings. Each variant passes 902 all-feature and 815 default
library tests, 28 doctests, the shared controls, six cache tests, eleven P4
tests, formatting, strict default/all-feature all-target Clippy and
warning-denied docs. Counts overlap; this is not the full workflow matrix.

All eight recorded layouts match. Memo inventories remain 1/1/2 for
retained/original/opaque controls, and ordinary-copy sharing remains 0/0.
Outlined and borrowed reuse the capacity-one input buffer in the surviving
radical control and release radical capacity on cancellation. Baseline
capacities are four in both cases. Atom capacities are zero throughout.

The [cost assessment](cost-assessment-v1.json) verifies all 18 preflights and
90 measurements. Builds preceded probes. Three unchanged suites used separate
timing/allocation executables and five fixed three-source blocks, with 4,096
timing or 1,000 allocation inputs per phase. Exact witnesses and representation
inventories match across variants. Each contrast has 68 timing and 68 memory
rows. These contrasts share samples and are not independent experiments.

| Fixed-trial observation | Outlined / baseline | Borrowed / baseline | Borrowed / outlined |
| --- | ---: | ---: | ---: |
| Slower in all five pairs | 4 | 6 | 4 |
| Slower in at least four | 18 | 15 | 14 |
| Faster in all five | 4 | 3 | 0 |
| Faster in at least four | 21 | 13 | 13 |
| Ratio of medians above one | 30 | 33 | 36 |
| Lower allocation calls/bytes | 6 | 6 | 0 |
| Lower peak requested bytes | 4 | 4 | 0 |
| Increased measured memory fields | 0 | 0 | 0 |
| Probe-size delta, bytes | +2,432–2,448 | +1,976–2,008 | −456 to −440 |

Borrowed is slower than baseline in all five pairs for rational
multiradical-square construction/first, quadratic-reciprocal construction/first,
quadratic multiradical-square cloned, and opaque-decline warm. Their median
paired ratios are 1.083040, 1.046091, 1.013163, 1.046058, 1.074120 and 1.010001.
All ranges remain in the assessment, including substantial variation.

For rational multiradical construction and first, ratios of medians are
1.654946 and 1.606915 despite paired medians 1.083040 and 1.046091. Both summaries
and all observations remain. Against outlined, borrowed is slower in every
pair for rational multiradical first and pi-parameter first/warm/cloned, with
paired medians 1.544787, 1.034141, 1.042463 and 1.024441. The adverse
radical-Laurent cloned comparison has timer-scale phase totals of about
16,000 ns over 4,096 inputs and is preserved without a practical-effect claim.

Borrowed is faster than baseline in all five pairs for signed-word
construction/first and wide-integer warm, with paired medians 0.977841,
0.960792 and 0.971437. Rational conjugate first/warm/cloned and quadratic
conjugate-then-affine first are faster in four pairs. Borrowed versus outlined
has thirteen at-least-four-faster phases and none faster in all five. All
favorable rows remain in the assessment.

Both product variants save 1,000 requested allocations and 64,000 allocated
bytes per 1,000 inputs for rational conjugate first/warm/cloned. Quadratic
conjugate-then-affine first/warm/cloned save 857 calls and 54,848 bytes.
Retained bytes match everywhere; four phases reduce peak requested bytes by
64. Every memory delta repeats five times. Borrowed and outlined counters
match in every row and pair. These Rust heap counters exclude native C heaps,
allocator bookkeeping and stack frames.

The fresh outlined control has four all-five-slower phases, compared with one
in its preceding closed trial. Directions and magnitudes vary across the
records; both remain. No previous sample is pooled or extended, and no earlier
rejection is erased. Ratios and directions are descriptive, without
significance or application-speedup claims.

The [static inspection](codegen-v1/protocol.json) reads only the existing
quadratic timing binaries. Disassembly is losslessly compressed, with separate
raw-output hashes. The baseline, outlined and borrowed coefficient symbols
contain 2,473, 2,800 and 2,993 bytes. Outlined's helper has 1,442 bytes and
borrowed's has 1,052: their combined symbol bytes fall by 197. Both entries
contain one emitted helper call. The polynomial symbol stays at 2,359 bytes.

The first coefficient stack-pointer subtractions are 248, 264 and 264 bytes;
both helpers subtract 104. These observations exclude pushes, callees and
dynamic behavior. They do not measure total/peak stack use or explain timings
causally. Internal helper jumps are not additional calls from the multiplier.

No native qualification, fixture assertion, matched native cost, full report
comparison or production CLI size was run for this rejected version. The
original limits remain 120 seconds of full-board worker CPU and 60 seconds of
neck check-body wall.

The current direct/outlined/borrowed helper series is closed. The next action
returns to native cost attribution at the unchanged endpoint/P4 parent:
re-read existing profiles and operation-count evidence beside the normalizer
and certificate-consumer code, then isolate work that can be removed before
proposing another optimization. If more attribution is needed, prepare a
separately frozen fixed profile or counter diagnostic on the full boards,
retaining the original assertion and distinguishing instrumented observations
from matched performance evidence. No equivalent helper variant is scheduled.

All current sources and samples remain frozen. Main runtime and the original
35 pending native paths are unchanged. The full plan remains active and
incomplete, with all 39 properties, 14 targets and crate future work in scope.
Individual property and target statuses remain unchanged.
