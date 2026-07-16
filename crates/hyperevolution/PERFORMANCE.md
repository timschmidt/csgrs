# HyperEvolution performance and reference audit

This audit covers every source in the README reference section. Timings came
from optimized local runs of the `fitness` benchmark on 2026-07-15 and are
comparative measurements, not portable latency guarantees.

## Retained results

| Path | Baseline median | Retained median | Result |
|---|---:|---:|---:|
| overlapping interval comparison | 90 ns | 67 ns | 25.6% faster |
| five-node GP validation | 41 ns | 15 ns | 63.4% faster |
| 32-gene first-improvement hill step | 54.216 us | 3.562 us | 93.4% faster |

Interval comparison now retains the two cross-endpoint orderings used by both
strict separation and overlap/unknown classification. GP validation computes
depth, node count, arity issues, and structural division issues in one traversal
without allocating a child vector at each node. First-improvement search lazily
constructs neighbors and stops after the first accepted improvement; best
improvement keeps eager full-neighborhood generation because an all-strategy
iterator regressed that aggregate path.

An optional `dispatch-trace` regression evaluates a rational GP expression and
an overlapping rational interval. It records exact dispatch/reducer work with
zero approximation and unknown-fact events.

## Reference mapping

### Yap, Towards Exact Geometric Computation

Genes, supported fitness, GP arithmetic, and interval endpoints remain
`hyperreal::Real`; accepted reports still require domain replay. Sparse GP batch
inputs now resolve directly by key: an absent input returns `MissingInput`
instead of being fabricated as exact zero. Selection also verifies that every
inspected fitness report names the candidate at the same aligned index. These
checks keep exact values attached to the right combinatorial candidate and
prevent a proposal adapter from inventing evidence.

### Moore, Kearfott, and Cloud, Introduction to Interval Analysis

Fitness intervals validate both enclosures before comparison and classify
strict separation, equality, overlap, invalid bounds, and undecidable endpoint
order separately. Reusing cross-endpoint orderings speeds the common overlap
case without midpoint or primitive-float fallback. Generated tests retain exact
minimize/maximize ordering for arbitrary disjoint integer intervals.

### Holland, Adaptation in Natural and Artificial Systems

The crate retains explicit populations, exact additive mutation, one-point
crossover, tournament index replay, structural diversity, and replay-gated
fitness reports. Random draws remain caller-owned and seed-bearing rather than
hidden inside selection. This audit additionally rejects candidate/report ID
misalignment. A full probabilistic genetic algorithm was not added because the
crate is the shared deterministic mechanics and evidence boundary, not a single
optimizer policy.

### Kirkpatrick, Gelatt, and Vecchi, Optimization by Simulated Annealing

Exact improvements and ties are classified directly. Worse moves remain a
named probabilistic proposal because evaluating the Boltzmann acceptance draw
is not exact replay evidence. The cooling schedule now requires positive
temperature and a ratio in `(0, 1]`; a ratio above one is explicitly invalid
instead of silently acting as heating. Seeded proposal engines can consume that
classification without promoting their draw to domain certification.

### Hoos and Stutzle, Stochastic Local Search

The hill climber exposes first/best improvement, evaluation budgets, plateau
policy, and tabu memory with deterministic neighborhood order and report-bearing
stop reasons. First improvement now generates only the neighbors it actually
inspects, producing the largest retained speedup. Best improvement still scans
the whole neighborhood as required. An all-strategy lazy iterator slowed the
best-improvement aggregate by about 5% and was replaced by the split design.

### Koza, Genetic Programming

Typed tree genomes validate input arity, depth, node budget, and structural
zero division before exact evaluation. The single-pass validation traversal
removes redundant tree walks and child-vector allocations. Sparse batch inputs
remain sparse so an unbound terminal cannot be reinterpreted as a zero-valued
terminal. Richer function sets, random tree generation, and evolutionary
population policy remain proposal-layer extensions.

### COCO/BBOB

`FitnessOracle`, `EvaluationCost`, deterministic cache keys, dependencies,
replay hooks, and the benchmark harness provide the accounting boundary needed
for black-box experiments. Hyper domain acceptance still requires replay even
when a surrogate screen says to promote a proposal. The official COCO platform
already owns its versioned noiseless, noisy, bi-objective, large-scale,
mixed-integer, constrained, and boxed suites, so those functions and result
archives were not copied into this crate.

## Considered but not retained

- One lazy neighbor iterator for both first and best improvement made the
  best-improvement-heavy aggregate about 5% slower. Lazy generation is retained
  only for first improvement; best improvement uses its measured eager path.
- Primitive-float midpoints would totally order overlapping intervals but would
  discard Moore-style enclosure semantics and were not introduced.
- Computing `exp(-delta/T)` internally would mix stochastic proposal policy
  with exact/domain replay. Worse annealing moves remain explicitly routed to a
  seeded probabilistic stage.
- Duplicating COCO's maintained test functions and data archives would create a
  second benchmark-version owner; the oracle/report boundary is sufficient.
