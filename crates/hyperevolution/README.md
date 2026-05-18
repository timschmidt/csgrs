<h1>
  hyperevolution
</h1>

`hyperevolution` is the exact-aware search and proposal layer for the Hyper ecosystem.
It defines candidate IDs, genomes, populations, replay policy, exact fitness reports,
lexicographic and Pareto comparisons, archives, and policy records for common
stochastic and local-search operators.

The crate does not own domain truth. It gives optimizers a shared way to keep proposals,
fitness, replay status, and acceptance evidence together.

## Hyper Ecosystem

`hyperevolution` is useful when a domain crate can generate candidates cheaply but needs
exact replay before accepting them.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact scalar genome and fitness
  values.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate decisions for
  candidate validation.
- [hypersolve](https://github.com/timschmidt/hypersolve): residual replay and constraint
  certification.
- [hypercurve](https://github.com/timschmidt/hypercurve),
  [hyperpath](https://github.com/timschmidt/hyperpath),
  [hyperdrc](https://github.com/timschmidt/hyperdrc),
  [hypercircuit](https://github.com/timschmidt/hypercircuit), and
  [hyperphysics](https://github.com/timschmidt/hyperphysics): domain crates that can
  certify accepted candidates.

## Typical Search Problems

Evolutionary and black-box optimizers often accept candidates because a sampled float
objective improved, even when constraints, geometry, or manufacturing rules were
evaluated by approximate surrogates. That makes accepted results hard to audit, hard to
reproduce, and easy to confuse with proof.

`hyperevolution` separates exploration from acceptance. A stochastic policy may propose
or rank candidates, but archive promotion can depend on exact fitness comparison and
domain replay status.

## Main Types

- `CandidateId`, `Genome`, `Candidate`, and `Population` describe search state.
- `FitnessValue`, `FitnessReport`, `FitnessDirection`, `FitnessComparison`, and
  `ParetoRelation` describe exact scalar, lexicographic, interval, and Pareto
  ordering.
- `ReplayPolicy` and `ReplayStatus` capture how exact acceptance is gated.
- `Archive` stores candidates with replay-aware acceptance status.
- `SelectionPolicy`, `MutationPolicy`, `CrossoverPolicy`, `HillClimbPolicy`,
  `HillClimbReport`, and `SimulatedAnnealingPolicy` record proposal settings and
  exact local-search outcomes without implementing every optimizer family.

## Precision Model

Genome values and fitness values use `Real`. Comparison helpers avoid collapsing exact
fitness to primitive floats before ordering. Interval fitness uses exact lower and upper
endpoints and reports overlap or invalid bounds explicitly rather than ranking by a
lossy midpoint. Replay status is explicit, so a candidate that is promising under a
sampled or approximate proposal can remain pending or unknown until the domain crate
certifies it.

## Performance Model

The crate is intentionally light. It stores policies, reports, and archive state without
forcing a single optimizer loop. That lets domain crates run fast approximate proposal
engines while retaining deterministic seeds, compact records, and exact comparison only
where promotion or audit requires it.

Future performance work should focus on batch replay scheduling, archive pruning, and
domain-specific caching rather than hiding approximations in the shared types.

## Current Status

Implemented today:

- exact candidate IDs, genomes, populations, and replay policy records;
- scalar, lexicographic, interval, and Pareto fitness reports over `Real` values;
- replay-gated archive records;
- deterministic first/best-improvement hill climbing over exact `Real` genomes;
- simulated-annealing acceptance classification that keeps worse-move probability
  handling as an explicit proposal stage;
- deterministic exact-best and explicit-index tournament selection plus exact
  additive mutation and one-point crossover for `Real` genomes;
- exact structural diversity reports based on genome equality and arity;
- typed `Real` GP expression genomes with arity, depth, node-budget, and
  structurally-zero-division validation before exact evaluation;
- black-box objective and surrogate-screening reports carrying fitness, cost,
  cache keys, construction dependencies, replay hooks, and promotion status;
- domain replay manifests and reports that name the Hyper crate responsible for
  certifying a memetic candidate;
- policy carriers for selection, mutation, crossover, hill climbing, and simulated
  annealing.

Known limits: large optimizer families are not implemented here yet. The crate is the
typed boundary those algorithms should use.

## Installation

```toml
[dependencies]
hyperevolution = "0.2.0"
```

For sibling checkouts:

```toml
[dependencies]
hyperevolution = { path = "../hyperevolution" }
```

## Usage

Use proposal helpers for search mechanics, then require exact replay for promotion:

```rust,ignore
use hyperevolution::{
    Candidate, CandidateId, FitnessDirection, FitnessReport, FitnessValue, Genome,
    MutationPolicy, ReplayPolicy, ReplayStatus, mutate_exact_delta,
};
use hyperreal::Real;

let candidate = Candidate {
    id: CandidateId::new("seed-0")?,
    genome: Genome { genes: vec![Real::from(2), Real::from(3)] },
    replay_policy: ReplayPolicy { seed: 42, require_exact_replay: true },
};

let mutated = mutate_exact_delta(
    &candidate.genome,
    &MutationPolicy::ExactDelta { gene: 0, delta: Box::new(Real::from(1)) },
)?;

let before = FitnessValue::Scalar(Box::new(Real::from(10)));
let after = FitnessValue::Scalar(Box::new(Real::from(8)));
assert_eq!(after.compare_total(&before, FitnessDirection::Minimize).is_better(), true);

let report = FitnessReport {
    candidate: candidate.id,
    value: after,
    replay: ReplayStatus::Accepted,
    evidence: vec!["domain replay accepted".into()],
};
```

For multi-objective work, use `FitnessValue::Pareto` and `Archive`; for symbolic or
memetic search, use `GpRealExpr`, surrogate-screen reports, and
`DomainReplayManifest` to keep the certifying crate named.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench fitness
```
