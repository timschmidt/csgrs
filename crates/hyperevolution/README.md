# Hyperevolution

Exact-aware candidate, fitness, archive, and replay carriers for optimization
in the Hyper stack.

Hyperevolution treats evolutionary and stochastic search as proposal
generation. A candidate may be generated or screened cheaply, but acceptance
retains exact fitness evidence and the owning domain’s replay status. The crate
does not own geometry, physics, circuit, packing, or manufacturing truth.

This README describes crate version `0.3.0`.

## Primary types

| Type | Role |
| --- | --- |
| `CandidateId`, `Genome`, `Candidate`, `Population` | Reproducible exact-valued search state |
| `FitnessValue`, `FitnessInterval`, `FitnessReport` | Scalar, lexicographic, Pareto, or interval objectives |
| `FitnessComparison`, `ParetoRelation` | Exact/partial ordering results |
| `Archive` | Replay-gated accepted or non-dominated reports |
| `SelectionReport`, `DiversityReport`, `HillClimbReport` | Deterministic search evidence |
| `GpRealExpr`, `GpValidationReport` | Budgeted typed real expression tree |
| `FitnessOracle`, `SurrogateScreen` | Explicit black-box and screening interfaces |
| `DomainReplayManifest`, `DomainReplayReport` | Owning-domain replay contract |

## Install

```toml
[dependencies]
hyperevolution = "0.3.0"
```

There are no default features. `dispatch-trace` forwards Hyperreal’s exact
dispatch instrumentation.

## Quick start

This checked example mutates an exact genome, compares exact fitness, and
archives an accepted replay.

<!-- quickstart:start -->
```rust
use hyperevolution::{
    Archive, Candidate, CandidateId, FitnessComparison, FitnessDirection, FitnessReport, Genome,
    Real, ReplayStatus, mutate_exact_delta,
};

fn main() {
    let seed = Candidate {
        id: CandidateId::new("seed").expect("non-empty id"),
        genome: Genome {
            genes: vec![Real::from(2), Real::from(3)],
        },
        proposal_seed: 42,
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
<!-- quickstart:end -->

Run it with:

```sh
cargo run --example basic
```

## Proposal and replay model

```text
candidate + deterministic seed
              │
 selection / variation / surrogate / oracle
              │
         fitness proposal
              │ owning-domain replay
              │
 accepted / rejected / unknown
              │
            Archive
```

Candidate IDs and fitness-report IDs must agree. Rejected or unknown replay
cannot silently enter the accepted archive.

## API guide

### Identity, fitness, and archives

- `CandidateId::new` creates a nonempty stable identity.
- `Genome`, `Candidate`, and `Population::push` store exact genes and proposal
  seeds.
- `FitnessReport::scalar` is the minimal report constructor.
  `FitnessValue` also represents lexicographic, Pareto, and interval objectives.
- `FitnessValue::{compare_total, compare_pareto}` reports exact ordering,
  partial ordering, or unknown comparison according to `FitnessDirection`.
- `FitnessInterval::{new, has_valid_bounds, compare}` preserves overlap and
  unknown endpoint ordering.
- `Archive::{insert_replayed, insert_non_dominated, reports}` enforces replay
  acceptance and exact comparison policy.

### Deterministic search mechanics

- `select_exact_best` selects from aligned candidate/report pairs.
- `select_tournament_by_indices` replays a caller-supplied tournament schedule;
  it performs no hidden random draw.
- `mutate_exact_delta` changes one exact gene under `MutationPolicy`.
- `crossover_one_point` applies an explicit split and `CrossoverPolicy`.
- `exact_structural_diversity` compares exact genomes and reports duplicate,
  distinct, or unresolved relations.
- `HillClimbPolicy::{first_improvement, best_improvement}` and
  `hill_climb_exact` run a bounded exact hill climb with replay reports.
- `classify_simulated_annealing_neighbor` classifies the acceptance decision
  for a supplied policy and proposal draw. Multiplicative cooling ratios must
  lie in `(0, 1]`.

### GP, oracles, and surrogate screening

- `GpRealExpr::{validate, eval, depth, node_count}` validates arity, input
  indices, and tree budgets before exact evaluation.
- `eval_gp_batch` applies the same validated expression to multiple exact
  inputs. Missing sparse inputs remain errors rather than zero.
- `FitnessOracle` and `evaluate_candidate_with_oracle` attach
  `BlackBoxEvaluationReport`, `EvaluationCost`, dependencies, cache key, and
  replay hook to an evaluation.
- `SurrogateScreen` returns a `SurrogateScreenReport` with stage and
  `SurrogateDecision`; it can reject or defer a proposal but cannot certify
  domain truth.
- `BlackBoxEvaluationReport::{is_promotable, requires_replay}` makes that
  boundary explicit.

### Domain replay

- `domain_replay_manifest` creates a `DomainReplayManifest` over one or more
  `DomainReplayTarget` values.
- `DomainReplayReport::{is_accepted, needs_followup}` summarizes returned
  domain evidence without discarding individual statuses.
- `EvaluationCacheKey`, `ConstructionDependency`, and `ReplayHook` keep cached
  evaluation tied to exact construction and replay context.

Replay targets include solver, curve, mesh, path, packing, DRC, physics, and
circuit owners. Hyperevolution does not reinterpret their reports.

## Guarantees and boundaries

- Genes and supported fitness values use `hyperreal::Real`.
- Scalar and lexicographic objectives have exact total comparison only when
  their values are orderable.
- Pareto and interval objectives preserve incomparability and overlap.
- Selection and variation helpers are deterministic given their explicit
  indices, split, delta, and seed.
- Approximate objectives, black-box evaluation, surrogate models, and
  stochastic choices remain named proposal stages.
- Search acceptance is distinct from domain replay acceptance.

Large optimizer families and probabilistic proposal engines are outside the
current crate; callers may implement them over these report-bearing seams.

## Feature flags

| Feature | Default | Purpose |
| --- | --- | --- |
| `dispatch-trace` | no | Hyperreal exact-dispatch instrumentation |

## Validation and performance

```sh
cargo fmt --all -- --check
cargo test --all-targets --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
cargo check --benches --all-features
```

Benchmark definitions and the per-reference performance audit are in
[PERFORMANCE.md](PERFORMANCE.md). Fuzz ownership is documented in
[fuzz/README.md](fuzz/README.md).

## References

- Yap, C. K. “Towards Exact Geometric Computation.” *Computational Geometry*
  7(1–2), 1997.
  [DOI: 10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).
- Moore, R. E., Kearfott, R. B., and Cloud, M. J. *Introduction to Interval
  Analysis*. SIAM, 2009.
  [DOI: 10.1137/1.9780898717716](https://doi.org/10.1137/1.9780898717716).
- Holland, J. H. *Adaptation in Natural and Artificial Systems*. MIT Press,
  1975/1992. [Publisher](https://mitpress.mit.edu/9780262581110/adaptation-in-natural-and-artificial-systems/).
- Kirkpatrick, S., Gelatt, C. D., and Vecchi, M. P. “Optimization by Simulated
  Annealing.” *Science* 220(4598), 1983.
  [DOI: 10.1126/science.220.4598.671](https://doi.org/10.1126/science.220.4598.671).
- Hoos, H. H., and Stützle, T. *Stochastic Local Search: Foundations and
  Applications*. Morgan Kaufmann, 2004.
  [Companion site](https://www.cs.ubc.ca/~hoos/SLS-Book/).
- Koza, J. R. *Genetic Programming*. MIT Press, 1992.
  [Publisher](https://mitpress.mit.edu/9780262527910/genetic-programming/).
- COCO Platform. *BBOB Test Suite*.
  [Official documentation](https://numbbo.github.io/coco/testsuites/bbob).

## Acknowledgements

Hyperevolution builds directly on
[Hyperreal](https://github.com/timschmidt/hyperreal). Domain replay is supplied
by the owning Hyper crates rather than duplicated here. The references above
inform search and evidence design without implying source-code derivation.

## License and contributing

Licensed under Apache-2.0 as declared in [Cargo.toml](Cargo.toml).

Bug reports should include candidates, fitness reports, replay manifest,
policies, explicit seeds/indices, and enabled features. Before proposing a
change, run formatting, focused tests, all targets/features, and strict Clippy.
