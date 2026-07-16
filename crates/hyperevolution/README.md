# hyperevolution

`hyperevolution` provides exact-aware candidate, fitness, archive, and replay
types for optimization in the Hyper stack. Search heuristics may propose or
screen candidates cheaply, but accepted results retain exact fitness and the
owning domain's replay status.

The crate does not own geometry, physics, circuit, or manufacturing truth. It
provides the shared boundary that keeps proposals, deterministic seeds,
fitness evidence, cache keys, and certification together.

## Installation

```toml
[dependencies]
hyperevolution = "0.3.0"
```

Use a sibling checkout during Hyper-stack development:

```toml
[dependencies]
hyperevolution = { path = "../hyperevolution" }
```

## Quick start

Mutate an exact genome, compare its fitness, and archive an accepted replay:

```rust
use hyperevolution::{
    Archive, Candidate, CandidateId, FitnessComparison, FitnessDirection,
    FitnessReport, Genome, Real, ReplayPolicy, ReplayStatus, mutate_exact_delta,
};

fn main() {
    let seed = Candidate {
        id: CandidateId::new("seed").expect("non-empty id"),
        genome: Genome {
            genes: vec![Real::from(2), Real::from(3)],
        },
        replay_policy: ReplayPolicy {
            seed: 42,
            require_exact_replay: true,
        },
    };
    let child = mutate_exact_delta(
        &seed,
        0,
        Real::from(-1),
        CandidateId::new("child").expect("non-empty id"),
    )
    .expect("gene 0 exists");

    let before = FitnessReport::scalar(seed.id, Real::from(4), ReplayStatus::Accepted);
    let after = FitnessReport::scalar(child.id, Real::from(1), ReplayStatus::Accepted);
    assert_eq!(
        after
            .value
            .compare_total(&before.value, FitnessDirection::Minimize),
        FitnessComparison::Better,
    );

    let mut archive = Archive::default();
    assert!(archive.insert_replayed(after));
}
```

## Core API

- `CandidateId`, `Genome`, `Candidate`, `Population`, and `ReplayPolicy` describe
  reproducible search state over `hyperreal::Real` genes.
- `FitnessValue` supports scalar, lexicographic, Pareto, and interval objectives.
  `compare_total`, `compare_pareto`, and `FitnessInterval::compare` preserve
  unknown or overlapping results instead of inventing a float ordering.
- `Archive` gates insertion on `ReplayStatus` and can retain a non-dominated
  accepted set.
- `select_exact_best`, `select_tournament_by_indices`, `mutate_exact_delta`,
  `crossover_one_point`, and `hill_climb_exact` provide deterministic mechanics
  without hidden random draws.
- `GpRealExpr` validates arity and tree budgets before exact evaluation.
- `FitnessOracle`, `SurrogateScreenReport`, `EvaluationCacheKey`, `ReplayHook`,
  and `DomainReplayManifest` carry black-box and domain-replay evidence.

## Precision and performance

Genes, supported fitness values, interval endpoints, and deterministic mutation
steps use `Real`. Approximate objectives and stochastic choices remain named
proposal stages. A candidate with rejected or unknown replay cannot silently
become accepted, and overlapping interval objectives remain unordered.

The crate stores compact policies and reports instead of forcing one optimizer
loop or expanding domain constraints into a global expression tree. Expensive
predicates, simulations, and residuals stay in their owning crates until replay.
This makes batch scheduling, archive pruning, and domain-specific caching the
natural performance levers.

Implemented today are exact scalar/lexicographic/interval/Pareto comparisons,
replay-gated archives, deterministic selection and variation, exact hill
climbing, simulated-annealing decision classification, structural diversity,
typed GP expressions, black-box/surrogate reports, and domain replay manifests.
Large optimizer families and probabilistic proposal engines are intentionally
outside the current implementation.

Selection rejects candidate/report ID misalignment, sparse GP inputs remain
missing rather than becoming zero, and multiplicative annealing cooling ratios
must lie in `(0, 1]`.

## References

- Yap, [*Towards Exact Geometric
  Computation*](https://doi.org/10.1016/0925-7721(95)00040-2), 1997.
- Moore, Kearfott, and Cloud, [*Introduction to Interval
  Analysis*](https://doi.org/10.1137/1.9780898717716), 2009.
- Holland, [*Adaptation in Natural and Artificial
  Systems*](https://mitpress.mit.edu/9780262581110/adaptation-in-natural-and-artificial-systems/),
  1975/1992.
- Kirkpatrick, Gelatt, and Vecchi, [*Optimization by Simulated
  Annealing*](https://doi.org/10.1126/science.220.4598.671), 1983.
- Hoos and Stutzle, [*Stochastic Local Search: Foundations and
  Applications*](https://www.cs.ubc.ca/~hoos/SLS-Book/), 2004.
- Koza, [*Genetic
  Programming*](https://mitpress.mit.edu/9780262527910/genetic-programming/),
  1992.
- COCO, [the BBOB test suite](https://numbbo.github.io/coco/testsuites/bbob).

Direct dependency: [hyperreal](https://github.com/timschmidt/hyperreal).
Replay domains include [hypersolve](https://github.com/timschmidt/hypersolve) ·
[hypercurve](https://github.com/timschmidt/hypercurve) ·
[hypermesh](https://github.com/timschmidt/hypermesh) ·
[hyperpath](https://github.com/timschmidt/hyperpath) ·
[hyperpack](https://github.com/timschmidt/hyperpack) ·
[hyperdrc](https://github.com/timschmidt/hyperdrc) ·
[hyperphysics](https://github.com/timschmidt/hyperphysics) ·
[hypercircuit](https://github.com/timschmidt/hypercircuit)

## Development

```sh
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
cargo bench --bench fitness
```

See [PERFORMANCE.md](PERFORMANCE.md) for the benchmark and per-reference audit.
