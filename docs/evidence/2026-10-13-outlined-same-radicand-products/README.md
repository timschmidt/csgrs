The outlined same-radicand product version is **rejected for runtime admission**.
All 42 scalar qualification commands passed, but cloned conjugate-then-affine
queries were slower than baseline in all five fixed pairs, with median paired
ratio 1.067268. Six phases retain the direct version's allocation savings; the
helper adds no measured memory saving over that control and increases probe
size. The [decision](decision-v1.json) retains every favorable and adverse
comparison. Main runtime is unchanged.

This experiment compares three fresh sources: the unchanged endpoint/P4
parent, the previously rejected direct version, and an outlined version. The
[review](scope-v1.json) proves the private helper's bounds and ownership contract.
Its single caller applies the original opaque-product guard, then establishes
singleton vectors and equal non-unit radicands. Neither input changes before
the helper. The left coefficient moves into the helper and the right remains
borrowed; no additional owner or scalar comparison is introduced relative to
the direct control.

The arithmetic suffix is unchanged apart from indentation and moving the
`Some` wrapper to the caller. Arithmetic order, zero skipping, first surviving
radicand payload, vector reuse and cancellation behavior remain. Removing the
helper and eligibility branch recovers the original general multiplier byte
for byte. Cache fields, memo publication, traversal budgets, mandatory retry
and the stronger-facts P4 contract remain inherited.

Each frozen source contains 10,136 files. Baseline and direct match their
previously qualified sources byte for byte. Only the declared algebra file
differs across variants. All 28 shared test functions, manifests and lockfiles
match. Source maps, deltas and inputs were committed before compilation.

The [qualification assessment](correctness-v1.json) verifies all 42 commands,
logs and source bindings. Each variant passes 902 all-feature and 815 default
library tests, 28 doctests, the shared controls, six cache tests, eleven P4
tests, formatting, strict default/all-feature all-target Clippy, and
warning-denied scalar docs. Counts overlap; this is not the full workflow
matrix.

All eight recorded type layouts match. Memo inventories remain 1/1/2 for
retained/original/opaque controls; ordinary-copy sharing remains 0/0. Direct
and outlined reuse the capacity-one input buffer in the surviving-radical
control and release radical capacity on cancellation. Baseline capacities
are four in both cases. Empty atom capacities are zero throughout. These
local observations do not establish total-memory or timing improvements.

The first cost runner failed during directory setup, before any build,
preflight or measurement. Its binary-output parent had an incorrect version
suffix. The [failure record](cost-setup-failure-v1.json) preserves that runner
and confirms all cost targets were absent. Version 2 fixes that parent and
uses fresh output paths, with identical qualified sources, probe code, sample
counts and block order. No measured sample was replaced.

The [cost assessment](cost-assessment-v2.json) verifies all 18 preflights and
90 measurements. All library and executable builds preceded probes. Three
unchanged suites used separate timing and allocation executables, five fixed
three-source blocks, and 4,096 timing or 1,000 allocation inputs per phase.
All exact witnesses and representation inventories match. Each contrast has
68 timing and 68 memory rows. Contrasts share samples and are not independent.

| Fixed-trial observation | Direct / baseline | Outlined / baseline | Outlined / direct |
| --- | ---: | ---: | ---: |
| Slower in all five pairs | 0 | 1 | 1 |
| Slower in at least four | 5 | 4 | 9 |
| Faster in all five | 4 | 6 | 3 |
| Faster in at least four | 23 | 21 | 18 |
| Ratio of medians above one | 15 | 17 | 35 |
| Lower allocation calls/bytes | 6 | 6 | 0 |
| Lower peak requested bytes | 4 | 4 | 0 |
| Increased measured memory fields | 0 | 0 | 0 |
| Probe-size increase, bytes | 1,968–1,984 | 2,432–2,448 | 456–480 |

Outlined conjugate-then-affine cloned queries have baseline/candidate medians
3,835,577 / 4,093,590 ns per 4,096-input phase. The paired ratio range is
1.014735–1.554612, with all five pairs slower. Negated-scaled-surd first,
multiradical-square first and canonical-integer construction are slower in
four pairs. Their median paired ratios are 1.038064, 1.029865 and 1.024243.
All ranges remain in the assessment, including substantial variation.

Against direct, outlined conjugate-product construction is slower in all
five pairs, median paired ratio 1.014867. Eight other phases are slower in
four pairs, including quadratic pi-transport first, opaque-decline queries
and mixed-field cases. Memory counters match between these two variants in
every phase and every pair.

Outlined is faster than baseline in all five pairs for rational conjugate
first/warm/cloned, pi-parameter warm/cloned, and opaque-linear-atom cloned.
Conjugate paired medians are 0.944441, 0.954266 and 0.958914. Against direct,
outlined is consistently faster for rational multiradical first and
pi-parameter warm/cloned. Every favorable row and its full range remain.

Both product variants save 1,000 requested allocations and 64,000 allocated
bytes per 1,000 inputs for rational conjugate first/warm/cloned. Quadratic
conjugate-then-affine first/warm/cloned save 857 calls and 54,848 bytes.
Retained bytes match everywhere; four phases reduce peak requested bytes by
64. Every memory delta repeats five times. Requested Rust heap counters
exclude native C heaps, allocator bookkeeping and stack frames.

The fresh direct control has zero all-five-slower phases, compared with eight
in its previous closed trial. Both records remain; this comparison neither
erases that rejection nor establishes a universal speed change. No historical
sample is pooled into this trial. Ratios and directions are descriptive,
without significance or application-speedup claims.

The [static inspection](codegen-v2/protocol.json) reads the three existing
quadratic timing binaries without compiling or rerunning probes. Baseline,
direct and outlined coefficient symbols contain 2,473, 4,161 and 2,800 bytes.
Outlined also has a 1,442-byte helper and one emitted call to it from the
multiplier. The polynomial multiplier stays at 2,359 bytes in all three.

The first stack-pointer subtractions in the coefficient multipliers are
248, 264 and 264 bytes, with 104 bytes in the outlined helper. These values
exclude pushes, callees and dynamic behavior; adding them does not measure
total or peak stack use. The static observations do not explain the timings
causally. Internal jumps in the helper's disassembly are recorded separately
from the multiplier's emitted call.

No native qualification, fixture assertion, matched native cost, complete
report comparison or production CLI size was executed for this rejected
version. The original 120-second board worker-CPU and 60-second neck
check-body-wall limits remain. The next review will consider a helper that
borrows already-checked fields, with arithmetic, payload identity, drop order
and bounds established before any new source is prepared. Any later trial
will have its own frozen sources and fixed samples.

All current sources, failures, samples and code-generation evidence remain
frozen. Main runtime and the original 35 pending native paths are unchanged.
The full plan remains active and incomplete, with all 39 properties, 14
targets and crate future work in scope. Individual ledger statuses remain.
