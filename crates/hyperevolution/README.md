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
- [hyperlattice](https://github.com/timschmidt/hyperlattice): vector and transform
  carriers used by domain crates whose candidates are searched here.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate decisions for
  candidate validation.
- [hypersolve](https://github.com/timschmidt/hypersolve): residual replay and constraint
  certification.
- [hypercurve](https://github.com/timschmidt/hypercurve),
  [hypertri](https://github.com/timschmidt/hypertri),
  [hypermesh](https://github.com/timschmidt/hypermesh),
  [hypervoxel](https://github.com/timschmidt/hypervoxel),
  [hyperpath](https://github.com/timschmidt/hyperpath),
  [hyperpack](https://github.com/timschmidt/hyperpack),
  [hyperparts](https://github.com/timschmidt/hyperparts),
  [hyperdrc](https://github.com/timschmidt/hyperdrc),
  [hypercircuit](https://github.com/timschmidt/hypercircuit), and
  [hyperphysics](https://github.com/timschmidt/hyperphysics): domain crates that can
  certify accepted candidates.
- [hyperbrep](https://github.com/timschmidt/hyperbrep): boundary-representation
  candidates for future geometry and manufacturing searches.
- [hypersdf](https://github.com/timschmidt/hypersdf): signed-distance and implicit-field
  candidates for future clearance and field searches.

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
- `GpRealExpr`, `GpValidationLimits`, `GpValidationReport`, and `eval_gp_batch` provide
  typed GP expression genomes with validation before exact evaluation.
- `FitnessOracle`, `BlackBoxEvaluationReport`, `SurrogateScreenReport`,
  `EvaluationCacheKey`, `ReplayHook`, and `DomainReplayManifest` keep opaque objective,
  surrogate, cache, and domain-replay evidence attached to candidates.
- `DiversityReport`, `DiversityRelation`, and `exact_structural_diversity` expose exact
  structural diversity without pretending approximate coordinate distance is proof.

## Precision Model

Genome values and fitness values use `Real`. Comparison helpers avoid collapsing exact
fitness to primitive floats before ordering. Interval fitness uses exact lower and upper
endpoints and reports overlap or invalid bounds explicitly rather than ranking by a
lossy midpoint. Replay status is explicit, so a candidate that is promising under a
sampled or approximate proposal can remain pending or unknown until the domain crate
certifies it.

Numerical explosion is controlled by keeping proposal mechanics separate from domain
certification. Genomes, GP expression trees, cache keys, replay hooks, surrogate stages,
and fitness reports are compact evidence carriers; expensive predicates, residuals, and
geometry checks stay in the owning domain crate until replay is requested.

## Performance Model

The crate is intentionally light. It stores policies, reports, and archive state without
forcing a single optimizer loop. That lets domain crates run fast approximate proposal
engines while retaining deterministic seeds, compact records, and exact comparison only
where promotion or audit requires it.

Future performance work should focus on batch replay scheduling, archive pruning, and
domain-specific caching rather than hiding approximations in the shared types.

The crate is deliberately friendly to approximate proposal engines: black-box or
surrogate stages can screen candidates, but archive promotion remains tied to exact
fitness comparison and accepted replay reports.

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
    Archive, Candidate, CandidateId, FitnessComparison, FitnessDirection, FitnessReport,
    FitnessValue, Genome, ReplayPolicy, ReplayStatus, mutate_exact_delta,
};
use hyperreal::Real;

let candidate = Candidate {
    id: CandidateId::new("seed-0")?,
    genome: Genome { genes: vec![Real::from(2), Real::from(3)] },
    replay_policy: ReplayPolicy { seed: 42, require_exact_replay: true },
};

let mutated = mutate_exact_delta(&candidate, 0, Real::from(1), CandidateId::new("seed-0-mut")?)?;

let before = FitnessValue::Scalar(Box::new(Real::from(10)));
let after = FitnessValue::Scalar(Box::new(Real::from(8)));
assert_eq!(
    after.compare_total(&before, FitnessDirection::Minimize),
    FitnessComparison::Better,
);

let report = FitnessReport {
    candidate: mutated.id,
    value: after,
    replay: ReplayStatus::Accepted,
    evidence: vec!["domain replay accepted".into()],
};

let mut archive = Archive::default();
assert!(archive.insert_replayed(report));
```

For multi-objective work, use `FitnessValue::Pareto` and `Archive`; for symbolic or
memetic search, use `GpRealExpr`, surrogate-screen reports, and
`DomainReplayManifest` to keep the certifying crate named.

```rust,ignore
use std::collections::HashMap;
use hyperevolution::{
    CandidateId, DomainReplayTarget, GpRealExpr, GpValidationLimits, domain_replay_manifest,
    eval_gp_batch,
};
use hyperreal::Real;

let expr = GpRealExpr::Add(
    Box::new(GpRealExpr::Input(0)),
    Box::new(GpRealExpr::Constant(Box::new(Real::from(2)))),
);
let validation = expr.validate(GpValidationLimits {
    input_arity: 1,
    max_depth: 4,
    max_nodes: 8,
});
assert!(validation.is_valid());

let mut inputs = HashMap::new();
inputs.insert(0, Real::from(3));
let values = eval_gp_batch(&[expr], &inputs);
assert_eq!(values[0].as_ref().unwrap(), &Real::from(5));

let manifest = domain_replay_manifest(
    CandidateId::new("seed-0-mut")?,
    DomainReplayTarget::Hyperpack,
    "layout-0",
);
assert_eq!(manifest.hook.domain, "hyperpack");
```

## References

- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry* 7.1-2
  (1997): 3-23.
- Moore, Ramon E. *Interval Analysis*. Prentice-Hall, 1966.
- Holland, John H. *Adaptation in Natural and Artificial Systems*. University of
  Michigan Press, 1975.
- Kirkpatrick, Scott, C. Daniel Gelatt, and Mario P. Vecchi. "Optimization by Simulated
  Annealing." *Science* 220.4598 (1983): 671-680.
- Hoos, Holger H., and Thomas Stutzle. *Stochastic Local Search: Foundations and
  Applications*. Morgan Kaufmann, 2004.
- Koza, John R. *Genetic Programming*. MIT Press, 1992.
- Hansen, Nikolaus, Anne Auger, Steffen Finck, and Raymond Ros. *Real-Parameter
  Black-Box Optimization Benchmarking 2009: Noiseless Functions Definitions*. INRIA,
  2009.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench fitness
```
